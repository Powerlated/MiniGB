#include <cstdio>
#include <SDL.h>

#include "MiniGB.h"

int main(int argc, char *argv[]) {
    printf("MiniGB");

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("MiniGB", 0, 0, 640, 576, SDL_WINDOW_SHOWN);
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    SDL_Delay(10000);

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}