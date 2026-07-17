"""Prepare an operator request for deterministic execution."""

import json
from dataclasses import dataclass
from typing import Optional

from omokai_executor import ArtifactKind, ExecutionPlan, MissionArtifactWriter
from omokai_interfaces import MissionProposal, PlanRequest, ValidatedMission
from omokai_mission import (
    CompiledRoute,
    Planner,
    RouteCatalog,
    ValidationResult,
    compile_mission,
    validate_proposal,
)

from .adapter import to_execution_plan


@dataclass(frozen=True)
class MissionPreparation:
    validated: ValidatedMission
    compiled: CompiledRoute
    execution_plan: ExecutionPlan


@dataclass(frozen=True)
class PreparationResult:
    request: PlanRequest
    proposal: MissionProposal
    validation: ValidationResult
    preparation: Optional[MissionPreparation]

    @property
    def accepted(self) -> bool:
        return self.preparation is not None


def prepare_mission(
    request: PlanRequest,
    mission_id: str,
    planner: Planner,
    catalog: RouteCatalog,
    artifact_writer: Optional[MissionArtifactWriter] = None,
) -> PreparationResult:
    """Ask a planner once, then prepare its exact proposal for execution."""

    proposal = planner.propose(request)
    return prepare_proposal(
        request=request,
        mission_id=mission_id,
        proposal=proposal,
        catalog=catalog,
        artifact_writer=artifact_writer,
    )


def prepare_proposal(
    request: PlanRequest,
    mission_id: str,
    proposal: MissionProposal,
    catalog: RouteCatalog,
    artifact_writer: Optional[MissionArtifactWriter] = None,
) -> PreparationResult:
    """Validate and compile one already-produced, immutable proposal.

    This entry point lets an operator approve a preview and pass that exact
    provider output to the live runner. Re-validation still happens at the
    execution boundary, but the LLM is not called a second time.
    """

    if proposal.request_id != request.request_id:
        raise ValueError('proposal request ID does not match the operator request')

    if artifact_writer is not None:
        artifact_writer.write_json(
            mission_id,
            ArtifactKind.PROPOSAL,
            {
                'request_id': proposal.request_id,
                'provider': proposal.provider,
                'content': proposal.content,
            },
        )

    validation = validate_proposal(
        proposal,
        route_directions=catalog.direction_policy(),
    )
    if artifact_writer is not None:
        artifact_writer.write_json(
            mission_id,
            ArtifactKind.VALIDATION,
            {
                'request_id': validation.request_id,
                'accepted': validation.accepted,
                'stage': validation.stage,
                'policy_version': validation.policy_version,
                'errors': validation.audit_records(),
            },
        )

    if not validation.accepted:
        return PreparationResult(request, proposal, validation, None)

    validated = validation.build_validated_mission(mission_id)
    compiled = compile_mission(validated, catalog)
    execution_plan = to_execution_plan(compiled, validated)
    preparation = MissionPreparation(validated, compiled, execution_plan)

    if artifact_writer is not None:
        artifact_writer.write_json(
            mission_id,
            ArtifactKind.ACCEPTED_MISSION,
            {
                'mission_id': validated.mission_id,
                'request_id': validated.request_id,
                'policy_version': validated.policy_version,
                'mission': json.loads(
                    json.dumps(dict(validated.mission.as_mapping()), allow_nan=False)
                ),
                'goals': [
                    {
                        'goal_id': goal.goal_id,
                        'kind': goal.kind.value,
                        'frame_id': goal.pose.frame_id,
                        'x': goal.pose.x,
                        'y': goal.pose.y,
                        'yaw': goal.pose.yaw,
                    }
                    for goal in execution_plan.goals
                ],
            },
        )

    return PreparationResult(request, proposal, validation, preparation)
