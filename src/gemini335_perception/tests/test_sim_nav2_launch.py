import time
import subprocess
import pytest

def test_nav2_active():
    """
    TDD Test for Nav2 Launch.
    It attempts to launch `sim_nav2.launch.py` and waits for it to become active.
    """
    launch_cmd = [
        "ros2", "launch", "gemini335_perception", "sim_nav2.launch.py",
        "map:=/home/lowsing/gemini335_ros2_ws_nav/maps/sim_map.yaml"
    ]
    process = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        # Nav2 takes a long time to start up, give it 30 seconds
        timeout = 30.0 
        start_time = time.time()
        
        costmap_advertised = False
        while not costmap_advertised and (time.time() - start_time) < timeout:
            if process.poll() is not None and process.returncode != 0:
                _, stderr = process.communicate()
                pytest.fail(f"Nav2 launch process failed early. Stderr: {stderr.decode('utf-8')}")
                
            # Check if /global_costmap/costmap topic is advertised
            try:
                topics = subprocess.check_output(["ros2", "topic", "list"]).decode('utf-8')
                if '/global_costmap/costmap' in topics.split():
                    costmap_advertised = True
                    break
            except Exception:
                pass
                    
            time.sleep(2.0)
            
        assert costmap_advertised, "Failed to see /global_costmap/costmap advertised by Nav2 within timeout"
    finally:
        process.kill()
        process.wait()
