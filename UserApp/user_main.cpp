#include "common_inc.h"
#include "user_main.h"



/**
 * @brief user main function (user code starts here)
 *
 * @param NULL
 */
void UserMain() {

    // Initialize Communication
    InitCommunication();

    // Initialize sentry robot parameters
    Global::sentry.Init();

    // Start Communication
    StartCommunication();

    // Launch all tasks
    LaunchAllTasks();
}