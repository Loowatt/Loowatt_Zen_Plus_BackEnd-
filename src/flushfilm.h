#pragma once

#include <Arduino.h>

class FlushFilm
{
    public:
    //constructor
    FlushFilm(signed long MAX_FILM);

    void init(signed long FilmCount, unsigned long FilmID); // FilmID: date when film was changed // Init is called whenever a new film is added on once at startup (eeprom)
    void updateFilmLeft(signed long Count);
    signed long getFilmLeft();

    private:
    long _FilmID;
    signed long _FilmLeft;
    signed long _MAX_FILM;
};