# Workflow

## Zasady

1. Kazda zmiana zaczyna sie od Issue.
2. Nie commitujemy bezposrednio do `main`.
3. Po zakonczeniu pracy tworzymy Merge Request do `develop`.
4. Kazdy Merge Request musi odnosic sie do Issue (np. `Closes #12`).

## Minimalne wymagania dla MR

- krotki opis zmiany i powod techniczny,
- link do Issue,
- informacja jak przetestowac zmiane.

## Aktualizacja repo

Przed rozpoczeciem pracy:
- `git pull`

Po zakonczeniu pracy:
- `git add .`
- `git commit -m "opis zmiany"`
- `git push`