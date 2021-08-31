#include "flushfilm.h"

FlushFilm::FlushFilm(signed long MAX_FILM)
{
    _MAX_FILM = MAX_FILM;
}

void FlushFilm::init(signed long FilmCount, unsigned long FilmID)
{
    _FilmLeft = FilmCount;
    _FilmID = FilmID;
}

void FlushFilm::updateFilmLeft(signed long Count)
{
    _FilmLeft = _FilmLeft - Count;
    if (_FilmLeft < 0)
    {
        _FilmLeft = 0;
    }
}

signed long FlushFilm::getFilmLeft()
{
    return _FilmLeft;
}


