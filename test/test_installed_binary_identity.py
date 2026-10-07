import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('binary_validation', REPO / 'scripts/dev_predictor/advisory_validation.py')
validation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validation)


class InstalledBinaryIdentityTest(unittest.TestCase):
    def test_actual_cmake_rpath_removal_and_unrelated_byte_change(self):
        with tempfile.TemporaryDirectory(prefix='iap_binary_identity_') as directory:
            root = Path(directory)
            source = root / 'module.c'
            source.write_text('int iap_identity_test(void) { return 7; }\n')
            built = root / 'module.so'
            subprocess.run(['cc', '-shared', '-fPIC', str(source), '-Wl,-rpath,' + str(root / 'build'),
                            '-o', str(built)], check=True, capture_output=True)
            installed = root / 'installed.so'
            installed.write_bytes(built.read_bytes())
            self.assertEqual(validation.installed_build_identity(installed, built)['match'], 'exact_bytes')
            script = root / 'install.cmake'
            script.write_text('file(RPATH_REMOVE FILE "' + str(installed) + '")\n')
            subprocess.run(['cmake', '-P', str(script)], check=True, capture_output=True)
            match = validation.installed_build_identity(installed, built)
            self.assertEqual(match['match'], 'cmake_build_rpath_removed')
            self.assertNotEqual(match['installed_sha256'], match['workspace_release_sha256'])
            changed = bytearray(installed.read_bytes())
            changed[-1] ^= 1  # Outside RPATH; retain the linker's GNU build ID.
            installed.write_bytes(changed)
            with self.assertRaisesRegex(ValueError, 'beyond an RPATH removal'):
                validation.installed_build_identity(installed, built)


if __name__ == '__main__':
    unittest.main()
