from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
import cv2
from MPC import start_mpc_simulation  # Adjust imports as needed
from ik_solver import solve_ik_ur5  # Adjust imports as needed

class RobotSimulationThread(QThread):
    update_joint_angles = pyqtSignal(list)

    def run(self):
        joint_angles = start_mpc_simulation()
        self.update_joint_angles.emit(joint_angles)

class ObjectDetectionThread(QThread):
    update_video = pyqtSignal(QImage)

    def run(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # Apply object detection here
            processed_frame = self.detect_objects(frame)

            # Convert frame to QImage
            rgb_image = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            step = channel * width
            qimage = QImage(rgb_image.data, width, height, step, QImage.Format_RGB888)
            self.update_video.emit(qimage)
        cap.release()

    def detect_objects(self, frame):
        # Replace this with the actual detection logic
        return frame

class RobotGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Control and Object Detection")
        self.setGeometry(100, 100, 1200, 800)

        # Layouts
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        # Robot Visualization
        self.robot_label = QLabel("Robot Visualization")
        left_layout.addWidget(self.robot_label)

        # Video Feed
        self.video_label = QLabel("Object Detection Feed")
        right_layout.addWidget(self.video_label)

        # Control Buttons
        start_button = QPushButton("Start")
        stop_button = QPushButton("Stop")
        start_button.clicked.connect(self.start_threads)
        stop_button.clicked.connect(self.stop_threads)

        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(start_button)
        bottom_layout.addWidget(stop_button)

        # Assemble Layouts
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)

        # Threads
        self.simulation_thread = RobotSimulationThread()
        self.simulation_thread.update_joint_angles.connect(self.update_robot_display)

        self.detection_thread = ObjectDetectionThread()
        self.detection_thread.update_video.connect(self.update_video_feed)

    def start_threads(self):
        self.simulation_thread.start()
        self.detection_thread.start()

    def stop_threads(self):
        self.simulation_thread.terminate()
        self.detection_thread.terminate()

    def update_robot_display(self, joint_angles):
        # Update robot visualization here
        self.robot_label.setText(f"Joint Angles: {joint_angles}")

    def update_video_feed(self, qimage):
        self.video_label.setPixmap(QPixmap.fromImage(qimage))

app = QApplication([])
gui = RobotGUI()
gui.show()
app.exec_()
