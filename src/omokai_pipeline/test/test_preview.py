import unittest

from omokai_pipeline.preview import run_preview


class PipelinePreviewTest(unittest.TestCase):
    def test_demo_reaches_success(self) -> None:
        lines, succeeded = run_preview()
        output = '\n'.join(lines)
        self.assertTrue(succeeded)
        self.assertIn('Execution plan: goals=9 speed=0.18m/s', output)
        self.assertIn('Final state: succeeded', output)
        self.assertIn('Final goal: home (home)', output)


if __name__ == '__main__':
    unittest.main()
