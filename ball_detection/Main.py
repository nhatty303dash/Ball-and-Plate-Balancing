import sys
import traceback
from PyQt6.QtWidgets import QApplication
from gui import MainWindow

def main():
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec())
    except Exception as e:
        # Print detailed error information to terminal
        print(f"Error: {str(e)}")
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()