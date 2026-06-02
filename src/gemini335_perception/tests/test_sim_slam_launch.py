import time
import subprocess
import pytest

def test_map_published():
    """
    TDD Test for SLAM Launch.
    It attempts to launch `sim_slam.launch.py` and waits for `/map`.
    """
    launch_cmd = ["ros2", "launch", "gemini335_perception", "sim_slam.launch.py"]
    process = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        timeout = 15.0  # Give SLAM 15 seconds to start
        start_time = time.time()
        
        map_advertised = False
        while not map_advertised and (time.time() - start_time) < timeout:
            if process.poll() is not None and process.returncode != 0:
                _, stderr = process.communicate()
                pytest.fail(f"Launch process failed early. Stderr: {stderr.decode('utf-8')}")
                
            # Check if /map topic is advertised using CLI
            try:
                topics = subprocess.check_output(["ros2", "topic", "list"]).decode('utf-8')
                if '/map' in topics.split():
                    map_advertised = True
                    break
            except Exception:
                pass
                    
            time.sleep(1.0)
            
        assert map_advertised, "Failed to see /map topic advertised by slam_toolbox within timeout"
    finally:
        process.kill()
        process.wait()
