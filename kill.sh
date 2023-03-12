kill -9 $(pidof gzserver)
kill -9 $(pidof gzclient)
kill -9 $(pgrep -f run.py)
kill -9 $(pgrep -f collision_publisher_node)