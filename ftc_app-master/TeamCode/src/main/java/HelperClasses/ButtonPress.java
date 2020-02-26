package HelperClasses;

/**
 * Get button presses.
 * The cringe is real
 */
public class ButtonPress {

    //All the last downs
    private static boolean gamepad1_a_last, gamepad1_b_last, gamepad1_x_last, gamepad1_y_last,
    gamepad1_dpad_up_last, gamepad1_dpad_down_last, gamepad1_dpad_right_last, gamepad1_dpad_left_last,
    gamepad1_right_bumper_last, gamepad1_left_bumper_last, gamepad1_left_stick_button_last,
    gamepad1_right_stick_button_last,

    gamepad2_a_last, gamepad2_b_last, gamepad2_x_last, gamepad2_y_last,
    gamepad2_dpad_up_last, gamepad2_dpad_down_last, gamepad2_dpad_right_last, gamepad2_dpad_left_last,
    gamepad2_right_bumper_last, gamepad2_left_bumper_last, gamepad2_left_stick_button_last,
    gamepad2_right_stick_button_last;


    //if they are pressed (result)
    private static boolean gamepad1_a_pressed,gamepad1_b_pressed,gamepad1_x_pressed,gamepad1_y_pressed,
            gamepad1_dpad_up_pressed,gamepad1_dpad_down_pressed,gamepad1_dpad_right_pressed,
            gamepad1_dpad_left_pressed, gamepad1_right_bumper_pressed, gamepad1_left_bumper_pressed,
            gamepad1_left_stick_button_pressed,gamepad1_right_stick_button_pressed,

            gamepad2_a_pressed, gamepad2_b_pressed, gamepad2_x_pressed, gamepad2_y_pressed,
            gamepad2_dpad_up_pressed, gamepad2_dpad_down_pressed, gamepad2_dpad_right_pressed, gamepad2_dpad_left_pressed,
            gamepad2_right_bumper_pressed, gamepad2_left_bumper_pressed, gamepad2_left_stick_button_pressed,
            gamepad2_right_stick_button_pressed;;

    public static boolean isGamepad1_a_pressed() {
        return gamepad1_a_pressed;
    }
    public static boolean isGamepad1_b_pressed() {
        return gamepad1_b_pressed;
    }
    public static boolean isGamepad1_x_pressed() {
        return gamepad1_x_pressed;
    }
    public static boolean isGamepad1_y_pressed() {
        return gamepad1_y_pressed;
    }
    public static boolean isGamepad1_dpad_up_pressed() {
        return gamepad1_dpad_up_pressed;
    }
    public static boolean isGamepad1_dpad_down_pressed() {
        return gamepad1_dpad_down_pressed;
    }
    public static boolean isGamepad1_dpad_right_pressed() {
        return gamepad1_dpad_right_pressed;
    }
    public static boolean isGamepad1_dpad_left_pressed() {
        return gamepad1_dpad_left_pressed;
    }
    public static boolean isGamepad1_right_bumper_pressed() {
        return gamepad1_right_bumper_pressed;
    }
    public static boolean isGamepad1_left_bumper_pressed() {
        return gamepad1_left_bumper_pressed;
    }
    public static boolean isGamepad1_left_stick_button_pressed() {
        return gamepad1_left_stick_button_pressed;
    }
    public static boolean isGamepad1_right_stick_button_pressed() {
        return gamepad1_right_stick_button_pressed;
    }
    public static boolean isGamepad2_a_pressed() {
        return gamepad2_a_pressed;
    }
    public static boolean isGamepad2_b_pressed() {
        return gamepad2_b_pressed;
    }
    public static boolean isGamepad2_x_pressed() {
        return gamepad2_x_pressed;
    }
    public static boolean isGamepad2_y_pressed() {
        return gamepad2_y_pressed;
    }
    public static boolean isGamepad2_dpad_up_pressed() {
        return gamepad2_dpad_up_pressed;
    }
    public static boolean isGamepad2_dpad_down_pressed() {
        return gamepad2_dpad_down_pressed;
    }
    public static boolean isGamepad2_dpad_right_pressed() {
        return gamepad2_dpad_right_pressed;
    }
    public static boolean isGamepad2_dpad_left_pressed() {
        return gamepad2_dpad_left_pressed;
    }
    public static boolean isGamepad2_right_bumper_pressed() {
        return gamepad2_right_bumper_pressed;
    }
    public static boolean isGamepad2_left_bumper_pressed() {
        return gamepad2_left_bumper_pressed;
    }
    public static boolean isGamepad2_left_stick_button_pressed() {
        return gamepad2_left_stick_button_pressed;
    }
    public static boolean isGamepad2_right_stick_button_pressed() {
        return gamepad2_right_stick_button_pressed;
    }


    public static void giveMeInputs(boolean gamepad1_a,
                                    boolean gamepad1_b,
                                    boolean gamepad1_x,
                                    boolean gamepad1_y,
                                    boolean gamepad1_dpad_up,
                                    boolean gamepad1_dpad_down,
                                    boolean gamepad1_dpad_right,
                                    boolean gamepad1_dpad_left,
                                    boolean gamepad1_right_bumper,
                                    boolean gamepad1_left_bumper,
                                    boolean gamepad1_left_stick_button,
                                    boolean gamepad1_right_stick_button,

                                    boolean gamepad2_a,
                                    boolean gamepad2_b,
                                    boolean gamepad2_x,
                                    boolean gamepad2_y,
                                    boolean gamepad2_dpad_up,
                                    boolean gamepad2_dpad_down,
                                    boolean gamepad2_dpad_right,
                                    boolean gamepad2_dpad_left,
                                    boolean gamepad2_right_bumper,
                                    boolean gamepad2_left_bumper,
                                    boolean gamepad2_left_stick_button,
                                    boolean gamepad2_right_stick_button){

        gamepad1_a_pressed = gamepad1_a && !gamepad1_a_last;
        gamepad1_b_pressed = gamepad1_b && !gamepad1_b_last;
        gamepad1_x_pressed = gamepad1_x && !gamepad1_x_last;
        gamepad1_y_pressed = gamepad1_y && !gamepad1_y_last;
        gamepad1_dpad_up_pressed = gamepad1_dpad_up && !gamepad1_dpad_up_last;
        gamepad1_dpad_down_pressed = gamepad1_dpad_down && !gamepad1_dpad_down_last;
        gamepad1_dpad_right_pressed = gamepad1_dpad_right && !gamepad1_dpad_right_last;
        gamepad1_dpad_left_pressed = gamepad1_dpad_left && !gamepad1_dpad_left_last;
        gamepad1_right_bumper_pressed = gamepad1_right_bumper && !gamepad1_right_bumper_last;
        gamepad1_left_bumper_pressed = gamepad1_left_bumper && !gamepad1_left_bumper_last;
        gamepad1_left_stick_button_pressed = gamepad1_left_stick_button && !gamepad1_left_stick_button_last;
        gamepad1_right_stick_button_pressed = gamepad1_right_stick_button && !gamepad1_right_stick_button_last;

        gamepad2_a_pressed = gamepad2_a && !gamepad2_a_last;
        gamepad2_b_pressed = gamepad2_b && !gamepad2_b_last;
        gamepad2_x_pressed = gamepad2_x && !gamepad2_x_last;
        gamepad2_y_pressed = gamepad2_y && !gamepad2_y_last;
        gamepad2_dpad_up_pressed = gamepad2_dpad_up && !gamepad2_dpad_up_last;
        gamepad2_dpad_down_pressed = gamepad2_dpad_down && !gamepad2_dpad_down_last;
        gamepad2_dpad_right_pressed = gamepad2_dpad_right && !gamepad2_dpad_right_last;
        gamepad2_dpad_left_pressed = gamepad2_dpad_left && !gamepad2_dpad_left_last;
        gamepad2_right_bumper_pressed = gamepad2_right_bumper && !gamepad2_right_bumper_last;
        gamepad2_left_bumper_pressed = gamepad2_left_bumper && !gamepad2_left_bumper_last;
        gamepad2_left_stick_button_pressed = gamepad2_left_stick_button && !gamepad2_left_stick_button_last;
        gamepad2_right_stick_button_pressed = gamepad2_right_stick_button && !gamepad2_right_stick_button_last;


        gamepad1_a_last = gamepad1_a;
        gamepad1_b_last = gamepad1_b;
        gamepad1_x_last = gamepad1_x;
        gamepad1_y_last = gamepad1_y;
        gamepad1_dpad_up_last = gamepad1_dpad_up;
        gamepad1_dpad_down_last = gamepad1_dpad_down;
        gamepad1_dpad_right_last = gamepad1_dpad_right;
        gamepad1_dpad_left_last = gamepad1_dpad_left;
        gamepad1_right_bumper_last = gamepad1_right_bumper;
        gamepad1_left_bumper_last = gamepad1_left_bumper;
        gamepad1_left_stick_button_last = gamepad1_left_stick_button;
        gamepad1_right_stick_button_last = gamepad1_right_stick_button;

        gamepad2_a_last = gamepad2_a;
        gamepad2_b_last = gamepad2_b;
        gamepad2_x_last = gamepad2_x;
        gamepad2_y_last = gamepad2_y;
        gamepad2_dpad_up_last = gamepad2_dpad_up;
        gamepad2_dpad_down_last = gamepad2_dpad_down;
        gamepad2_dpad_right_last = gamepad2_dpad_right;
        gamepad2_dpad_left_last = gamepad2_dpad_left;
        gamepad2_right_bumper_last = gamepad2_right_bumper;
        gamepad2_left_bumper_last = gamepad2_left_bumper;
        gamepad2_left_stick_button_last = gamepad2_left_stick_button;
        gamepad2_right_stick_button_last = gamepad2_right_stick_button;


    }
}
