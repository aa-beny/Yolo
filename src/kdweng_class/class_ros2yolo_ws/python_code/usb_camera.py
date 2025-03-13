import cv2


    

if __name__ == "__main__":
    # 打開 USB 相機，0 表示默認的相機設備
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("無法開啟相機")

    print("按 'q' 鍵退出程式")

    while True:
        # 讀取相機畫面
        ret, frame = cap.read()
        if not ret:
            print("無法讀取相機畫面")
            break

        # 顯示畫面
        cv2.imshow("USB Camera", frame)

        # 按下 'q' 鍵退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 釋放相機資源並關閉視窗
    cap.release()
    cv2.destroyAllWindows()
