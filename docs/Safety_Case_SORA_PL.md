# Studium Bezpieczeństwa: Ocena Ryzyka Operacji Specyficznych (SORA)

[Języki]: [EN](Safety_Case_SORA.md) | [PL](Safety_Case_SORA_PL.md) | [UA](Safety_Case_SORA_UA.md)

**Rola:** Specjalista ds. Bezpieczeństwa Lotniczego  
**Projekt:** DAS-SAR: Rozproszony Lotniczy Rój Poszukiwawczo-Ratowniczy  
**Wersja:** 1.0  
**Data:** 2026-01-14

## 1. Wstęp
Niniejszy dokument przedstawia studium bezpieczeństwa dla misji DAS-SAR Fazy 2: autonomicznej ewakuacji powietrznej ofiary wypadku przy użyciu rozproszonego roju transportowego. Ze względu na wysokie ryzyko związane z transportem ładunku ludzkiego, ocena ta jest zgodna z metodologią EASA SORA (Specific Operations Risk Assessment) w celu określenia klasy ryzyka naziemnego (GRC) i klasy ryzyka powietrznego (ARC), dążąc do klasyfikacji SAIL (Specific Assurance and Integrity Level) V lub VI.

## 2. Analiza Ryzyka: Ekstrakcja Ładunku Ludzkiego
Transport człowieka (80kg - 100kg) za pomocą rozproszonego roju dronów wprowadza kilka krytycznych czynników ryzyka, które różnią się od standardowych operacji cargo:

*   **Dotkliwość Obrażeń:** Katastrofa z udziałem ładunku ludzkiego jest z natury katastrofalna. W przeciwieństwie do ładunku obojętnego, człowiek jest wrażliwy na przyspieszenia, wibracje i orientację.
*   **Złożoność Systemu:** Użycie wielu fizycznych uwięzi zwiększa ryzyko splątania lub "spirali śmierci", jeśli jeden agent zachowa się nieobliczalnie.
*   **Niestabilność Dynamiczna:** Człowiek na noszach jest masą niejednorodną i przemieszczającą się. Ruch pacjenta może wywołać nieoczekiwane przesunięcia środka ciężkości (CG) i oscylacje wahadłowe w roju.
*   **Czynniki Środowiskowe:** Misje SAR często odbywają się w "miejskich kanionach" lub terenie górzystym, gdzie powszechne są zjawiska wielodrogowości sygnału GPS i nieprzewidywalne podmuchy wiatru.

## 3. Łagodzenie: Nadmiarowa Konfiguracja 6 Dronów
Głównym technologicznym środkiem łagodzącym ryzyko katastrofalnej awarii jest przejście z 4-dronowego prototypu na **6-dronową konfigurację fail-operational**.

### 3.1 Nadmiarowość Ciągu i Redukcja GRC
Rój został zaprojektowany tak, aby przetrwać całkowitą utratę dowolnego pojedynczego agenta (awaria 1 z 6) w trakcie lotu bez upuszczenia ładunku.

*   **Stan Nominalny:** Każdy z 6 dronów niesie ~17% całkowitego obciążenia (ok. 19kg z 115kg całkowitej masy startowej MTOM).
*   **Stan Awaryjny:** Jeśli silnik się spali, ESC ulegnie awarii lub bateria osiągnie stan krytyczny, logika sterowania roju (działająca w Rust dla zapewnienia niezawodności krytycznej dla bezpieczeństwa) redystrybuuje obciążenie. Pozostałe 5 dronów zwiększa swój ciąg, aby nieść po ~24kg każdy.
*   **Współczynnik Bezpieczeństwa:** Każdy dron o dużym udźwigu (wykorzystujący napęd T-Motor U15-II lub Hobbywing X9) jest oceniany na ciąg do 40kg. Zapewnia to stosunek ciągu do masy 2:1 nawet w stanie awaryjnym, gwarantując, że rój może utrzymać wysokość i wykonać kontrolowane lądowanie awaryjne.
*   **Wpływ na GRC:** Ten wysoki poziom nadmiarowości znacząco obniża klasę ryzyka naziemnego (GRC) poprzez zmniejszenie prawdopodobieństwa niekontrolowanego uderzenia. System przechodzi z paradygmatu "Fail-Safe" (bezpieczne rozbicie się) na "Fail-Operational" (kontynuowanie lotu).

## 4. Ograniczanie: Protokół Awaryjnego Odczepienia
Aby zapobiec sytuacji, w której pojedynczy ulegający awarii dron zagraża całemu rojowi, wdrożono "Protokół Awaryjnego Odczepienia" poprzez aktywne zarządzanie uwięzią.

### 4.1 Wykonanie Protokołu
W przypadku krytycznej awarii (np. awarii strukturalnej, pożaru lub całkowitej utraty kontroli), której nie można skompensować redystrybucją ciągu:

1.  **Generowanie Luzu:** Dron ulegający awarii (lub logika roju sterująca jego wyciągarką) próbuje stworzyć chwilowy luz w swojej uwięzi.
2.  **Zwolnienie Pirotechniczne/Mechaniczne:** Uruchamiany jest hak szybkiego uwalniania (sterowany serwem lub pirotechniczny) po stronie drona lub po stronie ramy rozporowej.
3.  **Autonomiczne Wyjście:** Uszkodzony dron odłącza się, aby zapobiec ściągnięciu pozostałych 5 dronów w dół lub wywołaniu "przeciągania liny", które mogłoby zdestabilizować nosze.
4.  **Ponowna Stabilizacja Roju:** Pozostałe 5 dronów natychmiast wchodzi w tryb stabilizacji o wysokim wzmocnieniu, aby skompensować nagłą zmianę wektora siły.

## 5. Poziomy Zapewnienia i Integralności (SAIL)
Zgodnie z wytycznymi SORA V2.5, operacja DAS-SAR jest celowana w poziom **SAIL V/VI**.

*   **Zapewnienie Oprogramowania:** Użycie języka **Rust** w pętli sterowania lotem i rozproszonej koordynacji zapewnia bezpieczeństwo pamięci i zapobiega typowym błędom współbieżności, które powodują zawieszenia systemu w rozwiązaniach opartych na C++.
*   **Niezależny FTS:** Każdy dron jest wyposażony w niezależny system przerywania lotu (FTS), który może być wyzwolony przez stację naziemną (GCS) lub pokładowy monitor "heartbeat", oddzielny od głównego autopilota PX4.
*   **Ograniczanie:** Protokół Awaryjnego Odczepienia służy jako krytyczna bariera ograniczająca, pozwalająca zamknąć skutki awarii agenta w utracie pojedynczej jednostki, a nie całego roju.
