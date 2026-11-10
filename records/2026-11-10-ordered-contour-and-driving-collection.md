# Partial contour replacement and menu7 collection
Base: 313512b. No whole-project restore. Original black-line path, controller gains and task sequence retained.
Ordinary white-road fallback tries ordered full contour arcs and locally aggregated row centers, then original row trace on invalid geometry. Missing borders never become artificial measured boundaries. Blue holes/branch topology fall back. Ring/committed exit selection explicitly retains original tracer.
Shipped contour_order_enabled=1; 0 restores old road extraction.
Menu7/1 repeats stationary capture every5s. Menu7/2 uses existing stand/start/stop workflow and asynchronous bounded recording. No other camera instance is launched for driving collection.
Driving records raw MJPG AVI, timestamp/command CSV, exact used configuration snapshots, source hashes, samples, and drop/error counts. Video nominal30fps is not elapsed time; CSV timestamps are authoritative. Only processed control frames are submitted; storage drops are explicit.
Offline replay compares white-road perception only and does not simulate closed-loop motion or claim accuracy from output rate.
Validation: 543 related tests passed, including contour winding reversal, blue-hole/missing-edge fallback, real video write/read and offline replay. bash -n and Python syntax passed. Physical ROS/chassis, throughput, region choice and steering behavior remain unvalidated.
Risk: contour topology may be ambiguous; new method falls back rather than fabricating edges. Recorder adds copying/encoding load despite nonblocking queue. Stop and review summary if recording errors occur. External video remains required to judge actual line crossings.
