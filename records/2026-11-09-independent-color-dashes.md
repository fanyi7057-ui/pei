# Independent colour evidence for ring dashes

Base a0d8b92 in E:/Two1. Existing uncommitted menu/capture changes are not
part of this change. Patrol masks, steering, speeds and task authorization
are unchanged. Only task dash detection takes the original resized BGR frame.

The colour detector builds its own neutral-dark candidates using absolute or
local contrast evidence, excludes blue hue, and requires a dark core. It does
not use the shared events morphology. It then reuses the existing geometry,
white-side, source-gap and entry/exit fork proof. Internal masks remain binary;
the distinguishing feature is independent source colour evidence.

Initial thresholds are explicit in detect_color_dashes: saturation <=100,
absolute V<=85, relative V<=175 with contrast>=25, core V<=110 on >=6% pixels,
21x21 local context at the existing processing resolution. These are starting
values, not calibrated physical reflectance. Dark blue and textured objects
remain adversarial cases requiring camera evidence.

Offline full-frame comparison (old hits / colour hits): recorded robot
50/44 of 1618 frames; normal handheld 28/25 of 1249; error/recovery 0/2 of 357.
The last two hits are frames 351/352 showing real dashes, labelled main dash
only, not an entry branch. Hit counts alone do not establish accuracy or
successful ring entry. No ground-truth-labelled recall claim is made.
Desktop colour detector time about 1.3--2.1 ms/frame; Orange Pi unmeasured.

Diagnostics show raw/shape/white-side/core rejection and blue pixel counts;
remaining geometry/gap decisions retain reason strings. Ordinary tracking
continues using its existing dark mask, with no extra filtering or rescue gate.
