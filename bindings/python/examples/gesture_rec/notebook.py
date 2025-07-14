from tkinter import *
from tkinter import ttk
import aditofpython as tof
import numpy as np
import cv2 as cv
from process import ProcessTab

SMALL_SIGNAL_THRESHOLD = 100

MODE_OPTIONS = [
    "0",
    "1",
    "2",
    "3"
]

# Global variables for the program
MODE_OPTIONS_CHOSEN = MODE_OPTIONS[0]
IS_STREAM_ON = False
IS_INITIALIZED = False
VAR_IP = "192.168.56.1"

DEPTH_THD = 1100  # 1000 mm

def mode_callback(selection):
    global MODE_OPTIONS_CHOSEN
    MODE_OPTIONS_CHOSEN = selection


def scale_callback(selection):
    global DEPTH_THD
    DEPTH_THD = (int)(selection)


def calc_process(depth_map):
    process = ProcessTab(depth_map)
    bounding_box = process.max_area.bbox
    return process.resultVar, (bounding_box[1], bounding_box[0]
                               ), (bounding_box[3], bounding_box[2])


class GestureDemo(Frame):

    def __init__(self, name='gesturedemo'):
        Frame.__init__(self, name=name)
        self.pack(expand=Y, fill=BOTH)
        self.master.title('Gesture Demo')
        self.resultVar = StringVar()
        self.box_start_point = (0, 0)
        self.box_end_point = (0, 0)
        self._create_main_panel()

    def _create_main_panel(self):

        main_panel = Frame(self, name='demo')
        main_panel.pack(side=TOP, fill=BOTH, expand=Y)

        # create the notebook
        nb = ttk.Notebook(main_panel, name='gesturedemo')
        nb.pack(fill=BOTH, expand=Y, padx=8, pady=8)
        self._create_video_tab(nb)

    def _create_video_tab(self, nb):
        # frame to hold content
        frame = Frame(nb, name='video')
        # widgets to be displayed on Video tab
        msg = ["Capture an image for processing"]
        lbl = Label(frame, justify=LEFT, anchor=N,
                    text=''.join(msg))
        lbl_frame = LabelFrame(frame, bg='red')

        MODE_OPTIONS_initial = StringVar()
        MODE_OPTIONS_initial.set(MODE_OPTIONS_CHOSEN)
        drop2 = OptionMenu(
            frame,
            MODE_OPTIONS_initial,
            *MODE_OPTIONS,
            command=mode_callback)

        depth_scale = Scale(
            frame,
            from_=550,
            to=2000,
            length=200,
            orient=HORIZONTAL,
            command=scale_callback)
        depth_scale.set(DEPTH_THD)
        btn_start = Button(frame, text='Start stream', underline=0,
                           command=lambda: self._start_video())
        btn_stop = Button(frame, text='Stop stream', underline=0,
                          command=lambda: self._stop_video())

        # position and set resize behaviour

        lbl.grid(row=0, column=0)
        Label(frame, text="IP address: ").grid(row=1, column=0)
        txt_box = Text(frame, width=30, height=1)
        txt_box.insert(1.0, VAR_IP)
        txt_box.grid(row=1, column=1)
        drop2.grid(row=2, column=0)
        drop2.grid(row=2, column=1)

        lbl_frame.grid(row=0, column=1, columnspan=4)
        btn = Button(
            frame,
            text='Initialize device',
            underline=0,
            command=lambda: self._init_dev(
                "ip:" +
                txt_box.get(
                    "1.0",
                    END)))
        btn.grid(row=2, column=2)
        Label(frame, text="Depth threshold [mm]:").grid(row=3, column=0)
        depth_scale.grid(row=3, column=1)
        btn_start.grid(row=6, column=0)
        btn_stop.grid(row=6, column=1)
        self.resultVar.set("How many fingers?")
        lbl_result = Label(frame, textvariable=self.resultVar, name='result')
        lbl_result.grid(row=7, column=0)

        nb.add(frame, text='Video', padding=2)

    def _init_dev(self, ip):
        global MODE_OPTIONS_CHOSEN
        global IS_INITIALIZED

        ip = ip.strip()

        print(ip)

        system = tof.System()
        print(
            "SDK version: ",
            tof.getApiVersion(),
            " | branch: ",
            tof.getBranchVersion(),
            " | commit: ",
            tof.getCommitVersion())

        self.cameras = []
        status = system.getCameraList(self.cameras, ip)
        if not status:
            return
        print("system.getCameraList()", status)

        status = self.cameras[0].initialize()
        if not status:
            return
        print("camera1.initialize()", status)

        modes = []
        status = self.cameras[0].getAvailableModes(modes)
        if not status:
            return
        print("camera1.getAvailableModes()", status)
        print(modes)

        camDetails = tof.CameraDetails()
        status = self.cameras[0].getDetails(camDetails)
        if not status:
            return
        print("camera1.getDetails()", status)
        print(
            "camera1 details:",
            "id:",
            camDetails.cameraId,
            "connection:",
            camDetails.connection)

        status = self.cameras[0].setMode(int(MODE_OPTIONS_CHOSEN))
        if not status:
            return
        print("camera1.setMode()", status)
        print("Chosen mode: ", MODE_OPTIONS_CHOSEN)

        IS_INITIALIZED = True

    def _start_video(self):
        global IS_STREAM_ON
        global IS_INITIALIZED
        global DEPTH_THD

        if not IS_STREAM_ON and IS_INITIALIZED:
            cam_details = tof.CameraDetails()
            status = self.cameras[0].getDetails(cam_details)
            if not status:
                print("system.getDetails() failed with status: ", status)

            # Enable noise reduction for better results
            self.cameras[0].setControl(
                "noise_reduction_threshold",
                str(SMALL_SIGNAL_THRESHOLD))

            status = self.cameras[0].start()
            print("self.cameras[0].start()", status)
            IS_STREAM_ON = True

            frame = tof.Frame()
            # Test frame, for metadata
            status = self.cameras[0].requestFrame(frame)
            if not status:
                print("cameras[0].requestFrame() failed with status: ", status)

            metadata = tof.Metadata
            status, metadata = frame.getMetadataStruct()

            width = metadata.width
            height = metadata.height

            # camera_range = np.power(2,16)
            depth_map = np.array(frame.getData("depth"), copy=False)
            depth_map[depth_map > DEPTH_THD] = DEPTH_THD
            camera_range = DEPTH_THD
            distance_scale = 255.0 / camera_range
            process_results = []

            while IS_STREAM_ON:
                # Capture frame-by-frame
                status = self.cameras[0].requestFrame(frame)
                if not status:
                    print(
                        "cameras[0].requestFrame() failed with status: ",
                        status)

                # Creation of the Depth image
                depth_map = np.array(frame.getData("depth"), copy=False)
                depth_map = depth_map[0: height, 0:width]
                depth_map[depth_map > DEPTH_THD] = DEPTH_THD
                depth_map = cv.flip(depth_map, 1)

                distance_scale = 255.0 / DEPTH_THD

                depth_map = distance_scale * depth_map
                depth_map = np.uint8(depth_map)

                # Image to display
                img = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)
                cv.rectangle(img, self.box_start_point,
                             self.box_end_point, (0, 255, 0), 15)
                cv.putText(
                    img,
                    self.resultVar.get(),
                    tuple(
                        coord +
                        5 for coord in self.box_start_point),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0,
                     0,
                     255))
                cv.namedWindow('Depth image', cv.WINDOW_AUTOSIZE)
                cv.imshow('Depth image', img)

                # if process_results != []:
                process_results = self.update_display(process_results)
                process_results.append(calc_process(depth_map))

                if cv.waitKey(1) == 27:
                    break

            cv.waitKey(1)
            cv.destroyWindow("Depth image")
            cv.waitKey(1)
            # Needs to be destroyed twice due to TKinter logic
            cv.destroyWindow("Depth image")
            cv.waitKey(1)

    def _stop_video(self):
        global IS_STREAM_ON
        if IS_STREAM_ON:
            IS_STREAM_ON = False
            cv.waitKey(10)
            status = self.cameras[0].stop()
            print("self.cameras[0].stop()", status)

    def update_display(self, process_results):
        to_delete = []
        for p in process_results:
            try:
                result, self.box_start_point, self.box_end_point = p
                self.resultVar.set(result)
            except Exception as e:
                self.resultVar.set("None")
                print("Exception:", e)
            finally:
                self.update()
                to_delete.append(p)

        return [p for p in process_results if p not in to_delete]


if __name__ == '__main__':
    GestureDemo().mainloop()
