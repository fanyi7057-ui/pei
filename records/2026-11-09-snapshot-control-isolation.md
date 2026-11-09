# Isolate optional snapshots from control

Robot log reproduced np.hstack dimension error: full-frame panels 240 rows,
road ROI 175 rows. Snapshot composition previously ran before worker start,
so the error escaped into control_exception and zeroed/reset control.

Road ROI is now placed at its original y origin in a full-frame blank canvas,
without geometric stretching. The panel labels its ROI. Missing masks receive
a labelled blank panel. Invalid dimensions are diagnostic failures only.

Copy owned buffers before worker launch. Conversion, annotation, concatenation
and disk I/O execute in a single bounded worker. Preparation/start exceptions
and worker exceptions are contained with 5-second warning limits. False
imwrite return is treated as a failed snapshot. No controller fault handling
is bypassed; this isolation is limited to optional diagnostics.

510 related tests passed including ROI/full/missing masks, preparation failure
with a real vision estimate, and false disk-write return. Actual Orange Pi
latency and startup line extraction are not established. Patrol masks,
thresholds, controller, speed and ring logic are unchanged.
