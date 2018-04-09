import darknet, cv2, glob
# Edit these 4 lines
net = darknet.load_net("/home/naoki/darknet/data/river/river2.cfg", "/home/naoki/darknet/backup/river/river2.backup", 0)
meta = darknet.load_meta("/home/naoki/darknet/data/river/river.data")
count = 0
for imgname in glob.glob("/home/naoki/darknet/river_photos/*.JPG"):
    r = darknet.detect(net, meta, imgname, thresh=0.35)
    img = cv2.imread(imgname)

    for k in range(len(r)):
        label = r[k][0]
        w = int(r[k][2][2])
        h = int(r[k][2][3])
        cx = int(r[k][2][0])
        cy = int(r[k][2][1])
        bx = int(cx-(w/2))
        by = int(cy-(h/2))
        conf = round(r[k][1],2)
        img = cv2.rectangle(img,(bx,by),
    						(bx+w,by+h),
    						(0,0,255),
    						2)
        thick = int((h + w) // 300)*3
        cv2.putText(img, label, (bx, by - 12),
            0, 1e-3 * img.shape[0],(0,0,255),thick//3)
        cv2.putText(img, str(conf), (bx, by - 36),
            0, 1e-3 * img.shape[0],(0,0,255),thick//3)
    # print r
    # cv2.imshow("f",img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    cv2.imwrite("res/"+str(count)+".JPG",img)
    count += 1

