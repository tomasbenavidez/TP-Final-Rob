import csv
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


class MclDiagnosticsCsvTest(unittest.TestCase):
    def test_writes_header_and_update_row(self):
        from tp_b_navigation.mcl_diagnostics import MclDiagnosticsCsv

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / 'mcl.csv'
            diagnostics = MclDiagnosticsCsv(path)
            diagnostics.write_update(
                stamp_sec=12.5,
                measurement_stamp_sec=12.25,
                event='laser',
                used=8,
                n_eff_before=20.0,
                n_eff_after=40.0,
                resampled=True,
                reset_weights=False,
                estimate_before=(1.0, 2.0, 0.1),
                estimate_after=(1.3, 2.4, 0.2),
                covariance_diag=(0.01, 0.02, 0.03),
                weight_max=0.05,
                log_likelihood_min=-8.0,
                log_likelihood_max=-1.0,
                accum_d=0.2,
                accum_a=0.1,
                laser_accum_d=0.2,
                laser_accum_a=0.1,
                detail='frame=rplidar_link',
            )
            diagnostics.close()

            rows = list(csv.DictReader(path.open()))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['event'], 'laser')
        self.assertEqual(rows[0]['stamp_sec'], '12.500000000')
        self.assertEqual(rows[0]['measurement_stamp_sec'], '12.250000000')
        self.assertAlmostEqual(float(rows[0]['delay_sec']), 0.25)
        self.assertEqual(rows[0]['used'], '8')
        self.assertEqual(rows[0]['resampled'], '1')
        self.assertEqual(rows[0]['reset_weights'], '0')
        self.assertAlmostEqual(float(rows[0]['delta_xy']), 0.5)
        self.assertEqual(rows[0]['detail'], 'frame=rplidar_link')


if __name__ == '__main__':
    unittest.main()
