import sensor
import time
import math

## Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
## The below thresholds track in general red/green things. You may wish to tune them...
thresholds = [
    (0, 100, 45, 127, -128, 127)
]

ball_thresholds = [(0, 100, 27, 58, 28, 80)]
y_goal_thresholds = [(0, 100, -20, 21, 30, 76)]
b_goal_thresholds = [(0, 100, 26, 78, -98, -51)]

## may pass up to 16 thresholds above. However, it's not really possible to segment any
## scene with 16 thresholds before color thresholds start to overlap heavily.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)  ## must be turned off for color tracking
sensor.set_auto_whitebal(False)  ## must be turned off for color tracking
clock = time.clock()

## only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
## returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
## camera resolution. Don't set "merge=True" because that will merge blobs which we don't want here.

while True:
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs(thresholds, pixels_threshold=1, area_threshold=100):
        ## values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255, 255, 255)) #bendable rectangle
            # img.draw_line(blob.major_axis_line(), color=(255, 255, 255))
            # img.draw_line(blob.minor_axis_line(), color=(255, 255, 255))
        ## values are stable all the time.
        img.draw_rectangle(blob.rect()) #standard rectangle
        img.draw_cross(blob.cx(), blob.cy())
        # note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=10
        )
    print(clock.fps())
