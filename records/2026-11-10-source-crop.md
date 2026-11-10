# Source crop for patrol and ring detection
Base 35f47a4. Partial edit only.
Live YAML crop_left_ratio=0.12, crop_right_ratio=0.12, crop_top_ratio=0.22; bottom retained.
Full camera image is cropped before common 320x240 processing. Black line, road contours, dash and zebra detection use the same cropped coordinates. Existing module ROIs remain relative to the cropped image.
640x480 input -> source view x=[76,564), y=[105,480), 488x375 pixels.
Runtime crop parameter changes reset geometry histories. Camera menu7 follows same crop, preserves full raw.png and adds cropped_color.png.
Dataclass defaults remain uncropped for legacy replay; shipped YAML enables the requested crop.
538 selected regression tests passed. Physical steering sensitivity, reduced bend visibility and false-trigger behavior remain to be verified. No speed/steering/entry-distance/probe-duration adjustments.
