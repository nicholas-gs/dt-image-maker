import cv2


def main():
    cap = cv2.VideoCapture()
    if cap is None:
        print("Video capture resource is none")
        return

    cap.open("""libcamerasrc ! video/x-raw, width=640, height=480,
        framerate=30/1 !videoconvert ! videoscale !
        clockoverlay time-format="%D %H:%M:%S" ! appsink""", cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("OpenCV cannot open resource!")
        return

    # take frame
    ret, frame = cap.read()
    # write frame to file
    cv2.imwrite('./test-opencv-gstreamer.jpg', frame)
    # release camera
    cap.release()


if __name__ == "__main__":
    main()
