"""Offline preview command tests."""

import io
import unittest
from contextlib import redirect_stdout

from omokai_mission import preview


class PreviewTest(unittest.TestCase):
    def test_default_prompt_accepts_and_compiles(self) -> None:
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = preview.main([])
        output = buffer.getvalue()

        self.assertEqual(0, code)
        self.assertIn('Validation: accepted=True', output)
        # two laps of the four-corner loop plus home.
        self.assertIn('Number of goals: 9', output)
        self.assertIn(
            'inspection_loop/segment1/counterclockwise/lap1/corner_sw',
            output,
        )
        self.assertIn('9. home', output)

    def test_single_lap_prompt(self) -> None:
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = preview.main(['Patrol the inspection loop once.'])
        output = buffer.getvalue()

        self.assertEqual(0, code)
        self.assertIn('Number of goals: 4', output)

    def test_render_is_deterministic(self) -> None:
        self.assertEqual(
            preview._render(preview.DEMO_PROMPT),
            preview._render(preview.DEMO_PROMPT),
        )


if __name__ == '__main__':
    unittest.main()
