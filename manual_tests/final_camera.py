from picamera2 import Picamera2, Preview
import cv2
import apriltag

# ===================== Config =====================
TAG_FAMILY = "tag25h9"
CAM_WIDTH = 640
CAM_HEIGHT = 480
# 锟斤拷示锟斤拷锟节达拷小锟斤拷锟斤拷锟皆讹拷锟藉）
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600
# ==================================================

picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (CAM_WIDTH, CAM_HEIGHT)})
picam2.configure(video_config)
picam2.start()

options = apriltag.DetectorOptions(families=TAG_FAMILY)
detector = apriltag.Detector(options)

print("========= AprilTag (tag25h9) Detector Started =========")
print("Press Q to quit")

# ===================== 锟斤拷锟斤拷锟睫革拷 =====================
# 1. 锟斤拷锟斤拷锟斤拷锟节ｏ拷锟斤拷锟斤拷为锟缴碉拷锟斤拷锟斤拷小
cv2.namedWindow("AprilTag Tag25h9 Detector", cv2.WINDOW_NORMAL)
# 2. 锟斤拷锟矫达拷锟节固讹拷锟竭寸（锟皆讹拷锟斤拷锟睫革拷锟斤拷锟斤拷锟斤拷锟斤拷郑锟?cv2.resizeWindow("AprilTag Tag25h9 Detector", DISPLAY_WIDTH, DISPLAY_HEIGHT)
# ====================================================

try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        tags = detector.detect(gray)

        for tag in tags:
            corners = tag.corners.astype(int)
            cv2.polylines(frame_bgr, [corners], True, (0, 255, 0), 2)

            cX = int(tag.center[0])
            cY = int(tag.center[1])
            cv2.circle(frame_bgr, (cX, cY), 4, (0, 0, 255), -1)

            # Custom ID mapping: 0=A, 1=B, 2=C
            if tag.tag_id == 0:
                display_text = "ID: A"
            elif tag.tag_id == 1:
                display_text = "ID: B"
            elif tag.tag_id == 2:
                display_text = "ID: C"
            else:
                display_text = f"ID: {tag.tag_id}"

            cv2.putText(
                frame_bgr,
                display_text,
                (cX - 10, cY - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        cv2.imshow("AprilTag Tag25h9 Detector", frame_bgr)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()