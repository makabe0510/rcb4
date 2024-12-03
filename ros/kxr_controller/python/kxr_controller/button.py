import rospy
from python_qt_binding.QtWidgets import QWidget, QGridLayout, QPushButton, QSizePolicy
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String, UInt16


class ActionButton(Plugin):
    def __init__(self, context):
        super(ActionButton, self).__init__(context)
        self.setObjectName("ActionButton")

        # Create QWidget for the GUI
        self._widget = QWidget()
        self._widget.setWindowTitle("Action Button GUI")

        # Grid Layout
        self.layout = QGridLayout(self._widget)
        self._widget.setLayout(self.layout)

        self.publisher = rospy.Publisher("/action_trigger", UInt16, queue_size=1)

        # Create buttons
        for row in range(4):
            for col in range(4):
                button_number = row * 4 + col + 1
                button = QPushButton(str(button_number))
                button.setSizePolicy(
                    QSizePolicy.Expanding, QSizePolicy.Expanding
                )  # Allow buttons to expand
                button.setMinimumSize(50, 50)
                button.clicked.connect(self.create_button_callback(button_number))
                self.layout.addWidget(button, row, col)

        # Add the widget to the context
        context.add_widget(self._widget)

    def create_button_callback(self, button_number):
        def callback():
            rospy.loginfo(f"Button {button_number} pressed")
            msg = UInt16(button_number)
            self.publisher.publish(msg)

        return callback
