package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

public class multiTheadExample {

    // Team: Here is a simple design pattern that you can use to run time consuming operations in parallel to your main loop

    //////////////////////////////////////////////////////////////////////
    // This approach launches time consuming code in a separate thread and returns control to the main loop.
    // In this pattern, you have two methods.  The first ones does the time consuming work.  It can be called from
    // a linear op mode if you don't mind waiting for it to finish.
    // The second method launches the first method in a new (separate) thread and immediately returns control to the
    // main loop.
    // This pattern requires a lock to make sure you don't launch more than one thread at a time.
    boolean somethingIsBusy = false;

    void doSomethingThatTakesAwhile (){
        // lock access to the method if it isn't already
        somethingIsBusy = true;
        // This method holds all your code that does whatever it is that takes awhile.
        // Maybe you are moving one servo, waiting for a second, and then moving another servo...
        // Or maybe you are operating a motor and waiting until it reaches a specific position
        // or...
        // Regardless, just do all the work in here and make sure you don't forget to check for opModeIsActive() in any loops you create
        // At the end, unlock access to the method
        somethingIsBusy = false;
    }

    void doSomethingThatTakesAwhileSeparateThread () {
        // This is the method that can be called to get the above method to run in a seperate thread
        // First we check to see if we are already running the other method
        if (somethingIsBusy == false) {
            somethingIsBusy = true; //lock access
            // Set up the seperate thread with a Runnable object whose run() method calls our doSomething method
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    doSomethingThatTakesAwhile();
                }
            });
            thread.start(); // launch the new thread but don't wait here for it to finish
        }
        // When this method exits, we return control to whoever called us, but our new thread continues to run
    }
}
