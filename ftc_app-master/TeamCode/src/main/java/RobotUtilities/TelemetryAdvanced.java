package RobotUtilities;

public class TelemetryAdvanced {
    public int size_x;
    public int size_y;

    public TelemetryAdvanced(int x_size, int y_size){
        size_x = x_size;
        size_y = y_size;
        init(size_x,size_y);
    }

    public native String getLine(int line);//this gets a line of the array from the c++ class to display
    public native void init(int size_x, int     size_y);//initializes our c++ arrays

    public native void putChar(int x, int y, int r, int g, int b, char myChar);//if you want to set a char

    public native void putCharField(double x, double y, char myChar);

    public native void clear();

    public native void drawLine(double x1, double y1, double x2, double y2, int r, int g, int b, char myChar);

    public native void drawField(double center_x, double center_y, double zoom);

    public native void drawRobot(double robotX, double robotY, double robotAng);
}
