"""Unified launcher: run.py / inference.py / sub_server.py share one CLI."""

import unittest

from src.app import main, parse_args


class AppLauncherTests(unittest.TestCase):
    def test_entry_scripts_share_main(self):
        import inference
        import run as run_mod
        import sub_server

        self.assertIs(run_mod.main, main)
        self.assertIs(inference.main, main)
        self.assertIs(sub_server.main, main)

    def test_parse_args_defaults(self):
        args = parse_args([])
        self.assertEqual(args.host, "0.0.0.0")
        self.assertEqual(args.port, 8080)
        self.assertFalse(args.yolo)
        self.assertFalse(args.no_sub)
        self.assertEqual(args.width, 640)
        self.assertEqual(args.height, 480)

    def test_old_flag_aliases_still_work(self):
        args = parse_args([
            "--web",
            "--sub",
            "--web-host", "127.0.0.1",
            "--web-port", "9090",
            "--camera-device", "2",
            "--left-device", "/dev/video0",
            "--right-device", "/dev/video2",
            "--yolo",
            "--backend", "hailo",
            "--timing",
            "--headless",
            "--tolerate-missing-devices",
        ])
        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 9090)
        self.assertEqual(args.camera_device, "2")
        self.assertEqual(args.left_device, "/dev/video0")
        self.assertEqual(args.right_device, "/dev/video2")
        self.assertTrue(args.yolo)
        self.assertEqual(args.backend, "hailo")
        self.assertTrue(args.timing)
        self.assertTrue(args.web)
        self.assertTrue(args.sub)

    def test_loopback_hostname_is_not_a_lan_url(self):
        from src.app import _is_usable_lan_ip

        self.assertFalse(_is_usable_lan_ip("127.0.0.1"))
        self.assertFalse(_is_usable_lan_ip("127.0.1.1"))
        self.assertFalse(_is_usable_lan_ip("169.254.12.34"))
        self.assertTrue(_is_usable_lan_ip("192.168.1.40"))

    def test_no_stero_typo_is_no_stereo(self):
        args = parse_args(["--no-stero"])
        self.assertTrue(args.no_stereo)

    def test_start_while_running_does_not_deadlock(self):
        import threading

        from src.inference_service import InferenceService

        hold = threading.Event()
        worker = threading.Thread(target=hold.wait, daemon=True)
        worker.start()
        svc = InferenceService()
        svc._owned = True
        svc._thread = worker
        svc._model_id = "detect"
        svc._backend = "hailo"
        result = []

        def call():
            result.append(svc.start("gate", "ncnn"))

        caller = threading.Thread(target=call, daemon=True)
        caller.start()
        caller.join(timeout=3.0)
        self.assertFalse(caller.is_alive(), "start() deadlocked on the service lock")
        self.assertTrue(result and result[0].get("ok"))
        self.assertEqual(svc._pending, ("gate", "ncnn"))
        hold.set()

    def test_start_same_model_does_not_spawn_second_loop(self):
        import threading

        from src.inference_service import InferenceService

        hold = threading.Event()
        worker = threading.Thread(target=hold.wait, daemon=True)
        worker.start()
        svc = InferenceService()
        svc._owned = True
        svc._thread = worker
        svc._model_id = "gate"
        svc._backend = "hailo"
        snap = svc.start("gate", "hailo")
        self.assertTrue(snap.get("ok"))
        self.assertIs(svc._thread, worker)
        hold.set()


if __name__ == "__main__":
    unittest.main()
