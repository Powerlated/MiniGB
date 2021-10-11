#include <cstdio>
#include <algorithm>
#include <SDL.h>
#include <fstream>
#include <ios>

extern "C" {
#include "MiniGB.h"
}

const int WIDTH = 640;
const int HEIGHT = 576;

const int GB_WIDTH = 160;
const int GB_HEIGHT = 144;
const int BYTES_PER_PIXEL = 4;

const double SECONDS_PER_FRAME = 70224.0 / 4194304.0;

double GetTime() {
    return (double)SDL_GetPerformanceCounter() / (double)SDL_GetPerformanceFrequency(); 
}

int main(int argc, char *argv[]) {
    printf("MiniGB\n");

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("MiniGB", 0, 0, WIDTH, HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, GB_WIDTH, GB_HEIGHT);

    SDL_SetWindowMinimumSize(window, WIDTH, HEIGHT);

    printf("Loading ROM...");
    // The first element of argv[] is the executable path
    std::ifstream file(argv[1], std::ios::binary | std::ios::ate);
    // Note: tellg() works only for binary files, not text files
    std::streamsize size = file.tellg(); 
    printf("Size (in bytes): %lld\n", size);

    file.seekg(0, std::ios::beg);
    
    GB_core_t *gbCore = (GB_core_t*)malloc(sizeof(GB_core_t));
    GB_init(gbCore);
    file.read((char*)gbCore->rom, size);
    if (file.fail()) {
        printf("Couldn't read ROM file\n");
        exit(1);
    }

    bool turbo = false;

    double nextFrameAt = 0;
    double fpsEvalTimer = 0;
    bool done = false;
    while (!done) {
        SDL_Event evt;
        while (SDL_PollEvent(&evt) != 0)
        {
            switch (evt.type)
            {
                case SDL_QUIT:
                    done = true;
                    break;
                case SDL_KEYUP:
                case SDL_KEYDOWN:
                    SDL_KeyboardEvent kb = evt.key;
                    bool pressed = kb.state == SDL_PRESSED;

                    switch (kb.keysym.sym){
                        case SDLK_SPACE:
                            turbo = pressed;
                            break;

                        case SDLK_z:
                            gbCore->button_b = pressed;
                            break;
                        case SDLK_x:
                            gbCore->button_a = pressed;
                            break;
                        case SDLK_BACKSPACE:
                            gbCore->button_select = pressed;
                            break;
                        case SDLK_RETURN:
                            gbCore->button_start = pressed;
                            break;

                        case SDLK_DOWN:
                            gbCore->button_down = pressed;
                            break;
                        case SDLK_UP:
                            gbCore->button_up = pressed;
                            break;
                        case SDLK_LEFT:
                            gbCore->button_left = pressed;
                            break;
                        case SDLK_RIGHT:
                            gbCore->button_right = pressed;
                            break;
                    }
                    break;
            }
        }

        double currentSec = GetTime();
        
        // Reset time if behind schedule
        if (currentSec - nextFrameAt >= SECONDS_PER_FRAME)
        {
            double diff = currentSec - nextFrameAt;
            // printf("Can't keep up! Skipping some time\n");
            nextFrameAt = currentSec;
        }

        if (currentSec >= nextFrameAt)
        {
            nextFrameAt += SECONDS_PER_FRAME;

            GB_run_to_next_frame(gbCore);

            int w;
            int h;
            SDL_GetWindowSize(window, &w, &h);

            double ratio = std::min((double)h / (double)HEIGHT, (double)w / (double)WIDTH);
            int fillWidth;
            int fillHeight;

            fillWidth = (int)(ratio * WIDTH);
            fillHeight = (int)(ratio * HEIGHT);

            SDL_Rect dest;
            dest.w = fillWidth;
            dest.h = fillHeight;
            dest.x = (int)((w - fillWidth) / 2);
            dest.y = (int)((h - fillHeight) / 2);

            // cool colors
            // uint32_t pixels[GB_WIDTH * GB_HEIGHT];
            // for (int i = 0; i < GB_WIDTH * GB_HEIGHT; i++) {
            //     pixels[i] = (i * 4543) | 0xFF000000;
            // }
            SDL_UpdateTexture(texture, NULL, GBPPU_get_display_screen_buffer(gbCore), GB_WIDTH * BYTES_PER_PIXEL);

            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, &dest);
            SDL_RenderPresent(renderer);
        }

        if (!turbo) {
            SDL_Delay(1);
        } else {
            GB_run_to_next_frame(gbCore);
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    printf("Quit");

    return 0;
}