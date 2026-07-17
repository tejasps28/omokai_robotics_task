# Mission Format

The planner emits JSON that conforms to schema version 1.1. A typical proposal
is:

```json
{
  "schema_version": "1.1",
  "action": "patrol",
  "route_id": "inspection_loop",
  "segments": [
    {
      "direction": "clockwise",
      "repetitions": 1
    },
    {
      "direction": "counterclockwise",
      "repetitions": 1
    }
  ],
  "speed_mps": 0.15,
  "return_home": true
}
```

## Fields

| Field | Meaning |
|---|---|
| `schema_version` | Mission contract version; currently `1.1` |
| `action` | Bounded operation; currently `patrol` |
| `route_id` | Symbolic route from the local catalog |
| `segments` | Ordered direction and repetition pairs |
| `speed_mps` | Requested linear speed, bounded by local policy |
| `return_home` | Append the catalog home pose after the route |

## Routes and directions

| Route | Allowed directions |
|---|---|
| `inspection_loop` | `clockwise`, `counterclockwise` |
| `aisle_sweep` | `forward`, `reverse` |
| `full_area_sweep` | `forward`, `reverse` |

The sum of segment repetitions must be between one and ten. The semantic
policy also limits speed to the simulated TurtleBot3 operating envelope.

## Validation boundary

Structural validation checks exact field names, types, enums, numeric bounds,
and additional properties. Semantic validation checks relationships that JSON
Schema alone does not express, such as whether a direction is valid for the
chosen route.

Only accepted missions receive a mission identity and reach the compiler.
Route poses, frame IDs, and home coordinates remain outside the model output.
