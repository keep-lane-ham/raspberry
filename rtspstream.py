# fast_rtsp_undist_h264_gop.py
import os, signal, numpy as np, cv2
from picamera2 import Picamera2
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

WIDTH, HEIGHT, FPS = 1456, 1088, 120
BITRATE = 50000            # kbps
KEYINT_SEC = 0.5          # ← 키프레임 주기(초). 0.5s=15프레임 @30fps
KEYINT = max(1, int(FPS * KEYINT_SEC))

# 보정 파라미터
CAL_W, CAL_H = 1456, 1088
FX0, FY0, CX0, CY0 = 1973.100022, 1968.166854, 680.320990, 520.991749
K1, K2, P1, P2, K3 = -0.57622269, 0.38526482, -0.00566925, 0.00209227, -0.13686068

cam = None
pts = 0
duration = int(1e9 / FPS)
map1, map2 = None, None

def on_need_data(appsrc, _):
    global pts
    if cam is None or map1 is None:
        return
    rgb = cam.capture_array()
    undist = cv2.remap(rgb, map1, map2, cv2.INTER_LINEAR)
    buf = Gst.Buffer.new_allocate(None, undist.size, None)
    buf.fill(0, undist.tobytes())
    buf.pts = pts
    buf.dts = pts
    buf.duration = duration
    pts += duration
    appsrc.emit("push-buffer", buf)

class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.set_shared(True)
        self.launch = (
            f"appsrc name=source is-live=true block=false do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
            "! queue leaky=downstream max-size-buffers=4 "
            "! videoconvert ! video/x-raw,format=I420 "
            "! queue leaky=downstream max-size-buffers=4 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={BITRATE} "
            f"   key-int-max={KEYINT} bframes=0 threads=0 sliced-threads=true byte-stream=true "
            f"   option-string=scenecut=0 "  # 정확히 {KEYINT} 주기로 IDR
            "! h264parse config-interval=1 "
            "! rtph264pay name=pay0 pt=96"
        )

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch)

    def do_configure(self, media):
        media.get_element().get_child_by_name('source').connect('need-data', on_need_data)

class GstServer(GstRtspServer.RTSPServer):
    def __init__(self):
        super().__init__()
        self.get_mount_points().add_factory("/cam", SensorFactory())
        self.attach(None)

def main():
    global cam, map1, map2, pts
    Gst.init(None)

    sx, sy = WIDTH/CAL_W, HEIGHT/CAL_H
    FX, FY, CX, CY = FX0*sx, FY0*sy, CX0*sx, CY0*sy
    K = np.float32([[FX,0,CX],[0,FY,CY],[0,0,1]])
    D = np.float32([[K1,K2,P1,P2,K3]])
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (WIDTH, HEIGHT), 0, (WIDTH, HEIGHT))
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (WIDTH, HEIGHT), cv2.CV_16SC2)

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (WIDTH, HEIGHT), "format": "RGB888"}))
    cam.start()
    pts = 0

    server = GstServer()
    print(f"RTSP URL: rtsp://<pi_ip>:8554/cam")

    loop = GLib.MainLoop()
    def stop(*_): loop.quit()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        loop.run()
    finally:
        cam.stop()

if __name__ == '__main__':
    main()