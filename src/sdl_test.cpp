//initializes gamepad
#include <SDL3/SDL.h>
#include <iostream>


int main(int argc, char* argv[]) {
    
    if (SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_EVENTS) < 0) {
    std::cerr << "SDL could not initialize: " << SDL_GetError() << "\n";
    return 1;
    }
    
    if (!SDL_HasGamepad()) { //checks if gamepad is connected
    std::cout << "No gamepads connected.\n"; //outputs if gamepad is not connected
    SDL_Quit(); //quits SDL
    return 0;
    }

    //opens gamepad
    int count = 0; //number of gamepads connected
    SDL_JoystickID *ids = SDL_GetGamepads(&count); //this will use our gamepad's id to read its sticks, also updates number of gamepads
    SDL_Gamepad* gamepad = NULL; 

    // Iterate over the list of gamepads
    for(int i = 0; i < count; i++) {
        SDL_Gamepad* gamepd = SDL_OpenGamepad(ids[i]);
        if(gamepad == NULL) {
            gamepad = gamepd;
        }
            
        std::cout << "Gamepad connected: " << SDL_GetGamepadName(gamepd) << "\n";
        
        // Close the other gamepads (of which there shouldn't be any)
        if(i > 0) {
            SDL_CloseGamepad(gamepd);
        }
    }
    if (!gamepad) {
        std::cerr << "Failed to open gamepad: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    //was runing into an issue with declaring bool running twice so I merged them together
    const int DEADZONE = 9000;
    bool running = true;
    SDL_Event event;

    std::cout << "Listening for gamepad input... (Press Ctrl+C or close window to quit)\n";

    while (running) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_EVENT_QUIT:
                    running = false;
                    break;

                case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
                    std::cout << "Button pressed: "
                              << SDL_GetGamepadStringForButton((SDL_GamepadButton)event.gbutton.button)
                              << "\n";
                    break;

                case SDL_EVENT_GAMEPAD_BUTTON_UP:
                    std::cout << "Button released: "
                              << SDL_GetGamepadStringForButton((SDL_GamepadButton)event.gbutton.button)
                              << "\n";
                    break;

                case SDL_EVENT_GAMEPAD_AXIS_MOTION: {
                    auto axis = (SDL_GamepadAxis)event.gaxis.axis;
                    int value = event.gaxis.value;

                    // Print axis movement and apply deadzone
                    if (axis == SDL_GAMEPAD_AXIS_LEFTX) {
                        if (value > DEADZONE)
                            std::cout << "Left stick →\n";
                        else if (value < -DEADZONE)
                            std::cout << "Left stick ←\n";
                    }

                    if (axis == SDL_GAMEPAD_AXIS_LEFTY) {
                        if (value > DEADZONE)
                            std::cout << "Left stick ↓\n";
                        else if (value < -DEADZONE)
                            std::cout << "Left stick ↑\n";
                    }

                    if (axis == SDL_GAMEPAD_AXIS_RIGHTX) {
                        if (value > DEADZONE)
                            std::cout << "Left stick →\n";
                        else if (value < -DEADZONE)
                            std::cout << "Left stick ←\n";
                    }

                    if (axis == SDL_GAMEPAD_AXIS_RIGHTY) {
                        if (value > DEADZONE)
                            std::cout << "Right stick ↓\n";
                        else if (value < -DEADZONE)
                            std::cout << "Right stick ↑\n";
                    }

                    break;
                }
            }
        }

        SDL_Delay(16); // ~60Hz
    }

    SDL_CloseGamepad(gamepad);
    SDL_Quit();
    return 0;
}