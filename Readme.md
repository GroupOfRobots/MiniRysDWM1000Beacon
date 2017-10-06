# RysBeacon

[EN] This code implements (*will implement*) tasks of DWM1000-based beacons (anchors) on STM Nucleo F303K8 for MiniRyś project

[PL] Ten kod realizuje (*będzie realizować*) zadania bazującej na DWM1000 latarni ("kotwicy") na STM Nucleo F303K8 dla projektu MiniRyś.

## Struktura projektu

Kod bazowy został podwójnie wygenerowany w `STMCubeMX` - raz dla środowiska GCC/Make, raz dla SystemWorkbench, następnie plik `Makefile` został ręcznie dostosowany do kodu dla SystemWorkbench.

Dodany został kod API dla modułów DW1000 (katalog `decadriver`), plik `dwm_platform.c` implementujący funkcje używane przez w/w API, zaś w pliku `main.c` i `main.h` dodany został (minimalnie przerobiony) przykładowy kod dla DW1000, dokładniej przykłady `01a` (nadajnik), `02a` (odbiornik) i `06b` (responder dla pomiaru odległości), przełączane stałymi kompilacji ustalanymi w `Makefile` (lub właściwościach projektu SW), odpowiednio: `DWM_TRANSMITTER`, `DWM_RECEIVER` i `DWM_RESPONDER`.

## Aktualny status

Nie działa.

Dla nadajnika: ramki nie są odbierane na drugim urządzeniu (MiniRyś - BeagleBone Green Wireless + DWM1000; możliwa była komunikacja MiniRyś <-> MiniRyś).

Dla odbiornika: pomimo braku drugiego uruchomionego urządzenia latarnia ciągle (tzn. po jednym od razu kolejny) sygnalizuje odbieranie ramek.

Kod jest poprawny formalnie (kompiluje się), zgodny z przykładami i API. Dodane zostało sygnalizowanie statusu za pomocą diody LED na płytce Nucleo (`LD3`) - nie są raportowane żadne błędy, tzn. wszystko wskazuje na prawidłową pracę systemu.

Komunikacja SPI jest weryfikowana na etapie inicjalizacji - z DW1000 odczytywany jest rejestr `0x01`, zawierający identyfikator urządzenia, a następnie zawartość rejestru jest porównywana ze stałą (`0xDECA0130`) - w przypadku niezgodności nastąpiło by zakończenie programu i raportowanie błędu (co się nie dzieje).

Nie działają diody RX/TX - nie wiadomo, czy jest to kwestia złej konfiguracji elektrycznej, w kodzie czy też nieprawidłowego działania kodu głównego (co skutkowałoby brakiem rzeczywistego nadawania/odbioru). Diody ustawiane były (aktualnie zakomentowanym) poleceniem `dwt_setleds(0x01);` z API DW1000.

## Proponowane dalsze prace
* Zweryfikować połączenia elektryczne
* Zweryfikować prawidłowe podłączenie diód RX/TX
* Zweryfikować prawidłowe zasilanie modułu DWM1000 pod obciążeniem
* Ustawić port szeregowy przez USB i dodać dokładniejsze raportowanie stanu tamże
