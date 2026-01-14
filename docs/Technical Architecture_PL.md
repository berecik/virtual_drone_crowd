# Architektura Techniczna: Rozproszony Lotniczy System Poszukiwawczo-Ratowniczy (DAS-SAR)

[Języki]: [EN](Technical%20Architecture.md) | [PL](Technical%20Architecture_PL.md) | [UA](Technical%20Architecture_UA.md)

## 1. Przegląd Systemu
System DAS-SAR wykorzystuje **Rozproszony System Podnoszenia (DLS)** do transportu ciężkich ładunków, takich jak ofiary wypadków, przy użyciu roju skoordynowanych dronów. Ta architektura oddziela całkowity udźwig od rozmiaru poszczególnych agentów, umożliwiając wykorzystanie dronów przenośnych przez człowieka do misji transportu ciężkich ładunków.

## 2. Topologia Roju

### 2.1 Faza 1: Skalowany Prototyp
*   **Wielkość Floty:** 4 Jednostki (Quadcoptery).
*   **Cel:** Walidacja praw sterowania i kooperatywnej estymacji stanu.

### 2.2 Faza 2: Rój w Pełnej Skali
*   **Wielkość Floty:** 6 Jednostek (Współosiowe Octocptery/X8) + 1 Agent Rezerwowy.
*   **Platforma:** Niestandardowe ramy o dużym udźwigu zoptymalizowane pod kątem transportu rozproszonego.
*   **Konfiguracja:** Topologia gwiazdy uwięzionej, gdzie każdy agent jest połączony z ładunkiem poprzez aktywny system wyciągarek.

### 2.3 Fizyka Nadmiarowości 6 Agentów
Rój Fazy 2 jest wyraźnie zdefiniowany jako minimum **6 agentów**, aby zapewnić zdolność **fail-operational** (kontynuację pracy po awarii).
*   **Sterowanie 6-DOF:** Ładunek na uwięzi wymaga minimum 6 liniowo niezależnych wektorów naciągu, aby osiągnąć pełną sterowalność nad jego 6 stopniami swobody (translacja 3D i rotacja 3D).
*   **Bezpieczna Nadmiarowość:** Wykorzystując 6 agentów w konfiguracji geometrycznej unikającej współpłaszczyznowości uwięzi, system utrzymuje pełną kontrolę 6-DOF nad ładunkiem. W przypadku awarii pojedynczego agenta, pozostali agenci mogą redystrybuować swoje wektory ciągu. Chociaż utrata jednego agenta zmniejsza całkowity udźwig, "minimum 6 agentów" zapewnia, że rój nie traci zdolności do stabilizacji orientacji lub pozycji ładunku, zapobiegając katastrofalnym oscylacjom lub utracie kontroli podczas manewrów awaryjnych.

## 3. Rozproszona Alokacja Sterowania
Sercem inteligencji DAS-SAR jest warstwa **Rozproszonej Alokacji Sterowania (DCA)**, która przekłada pożądany ruch ładunku na wartości zadane dla poszczególnych agentów.

### 3.1 Obliczanie Wektora Ciągu
Kontroler roju (uruchamiający algorytm oparty na konsensusie na modułach Jetson Orin) oblicza wymagany **Wypadkowy Wrench** ($\mathbf{W}_{req}$) potrzebny do poruszania ładunkiem wzdłuż jego trajektorii:
$$\mathbf{W}_{req} = [F_x, F_y, F_z, \tau_x, \tau_y, \tau_z]^T$$

### 3.2 Logika Alokacji
Algorytm DCA rozdziela ten wrench pomiędzy $N$ aktywnych agentów:
1.  **Optymalizacja:** Rozwiązuje problem programowania kwadratowego z ograniczeniami (QP), aby znaleźć optymalny naciąg $T_i$ dla każdej uwięzi.
2.  **Ograniczenia:**
    *   $T_{min} \leq T_i \leq T_{max}$ (Zapewnienie, że uwięzi nigdy nie poluzują się, a silniki pozostaną w granicach parametrów).
    *   Ograniczenia geometryczne oparte na aktualnych pozycjach względnych dronów względem ładunku.
3.  **Zarządzanie Nadmiarowością:** Jeśli agent zgłosi awarię lub niski stan baterii, DCA natychmiast przelicza macierz alokacji dla $N-1$ agentów, zwiększając obciążenie pozostałych jednostek w celu utrzymania wymaganego $\mathbf{W}_{req}$.

## 4. Specyfikacje Sprzętowe

### 4.1 Napęd Fazy 2 (Rój "Makro")
Aby obsłużyć wymaganie 100kg+ ładunku z wysokim marginesem bezpieczeństwa, Faza 2 wykorzystuje przemysłowe systemy napędowe.
*   **Silniki:** **T-Motor U15 II** lub **Hobbywing XRotor X9 Plus**.
*   **Konfiguracja:** Układ współosiowy X8 dla wewnętrznej nadmiarowości silników.
*   **Stosunek Ciągu do Masy:** Docelowy stosunek 2:1 przy nominalnym obciążeniu, aby zapewnić zdolność odzyskiwania sprawności podczas awarii agenta.

### 4.2 Aktywne Mechanizmy Wyciągarek
Każdy agent Fazy 2 jest wyposażony w **Aktywny Mechanizm Wyciągarki** do kontroli długości uwięzi.
*   **Dynamiczne Tłumienie:** Wyciągarka aktywnie dostosowuje długość uwięzi, aby tłumić oscylacje ładunku i kompensować przemieszczenia wywołane podmuchami wiatru.
*   **Zmienna Geometria:** Pozwala rojowi rozszerzać lub kurczyć swój promień w locie, aby nawigować przez wąskie przejścia (np. między budynkami lub drzewami).
*   **Awaryjne Odczepienie:** Zintegrowany szybki przecinak kabli i zwalniacz serwo dla natychmiastowego oddzielenia awaryjnych agentów.

## 5. Stos Oprogramowania
*   **Middleware:** Eclipse Zenoh (sieć mesh Peer-to-Peer).
*   **Logika Sterowania:** Rust (krytyczne dla bezpieczeństwa pętle sterowania lotem).
*   **Autopilot:** PX4 działający na Pixhawk 6C / Cube Orange.
*   **Estymacja Stanu:** Wizualno-Inercyjna Odometria (VIO) fuzowana z RTK-GNSS.
