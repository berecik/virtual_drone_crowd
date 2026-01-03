# Dokumentacja Systemu: Rozproszony Lotniczy System SAR (DAS-SAR)

**Autorzy:** beret@hipisi.org.pl & Marysia Software Limited (ceo@marysia.app, https://marysia.app)  
**Domena:** app.marysia.drone  
**Wersja:** 1.1 (2026-01-03)

## 1. Podsumowanie Menedżerskie

Projekt Rozproszonego Lotniczego Systemu SAR (DAS-SAR), znany również jako **Virtual Drone Crowd**, rozwiązuje wyzwania logistyczne operacji poszukiwawczo-ratowniczych (SAR) poprzez wprowadzenie **Rozproszonego Systemu Nośnego (DLS - Distributed Lift System)**. Dzięki oddzieleniu udźwigu od rozmiaru pojedynczego drona, rój przenośnych jednostek może współpracować w celu transportu ciężkich ładunków (ponad 100 kg). Rozwiązanie to umożliwia szybkie rozmieszczenie w trudnodostępnych terenach, takich jak góry czy kaniony miejskie, gdzie tradycyjne ciężkie śmigłowce nie mogą dotrzeć.

## 2. Podstawy Teoretyczne

### 2.1 Fizyka Sprzężonych Układów Ładunków Podwieszonych
Podstawą inżynieryjną systemu jest kooperacyjna manipulacja lotnicza. Jednostki są fizycznie połączone z ładunkiem za pomocą elastycznych uwięzi. Stan systemu jest wektorem wypadkowym sił naciągu przyłożonych przez $N$ jednostek. Pozwala to na "sterowanie geometryczne", w którym orientacja ładunku może być kontrolowana niezależnie od jego trajektorii — co jest kluczowe przy manewrowaniu noszami w złożonym otoczeniu (np. pod koronami drzew).

### 2.2 Strategie Sterowania: Admitancja vs. Impedancja
Standardowe kontrolery pozycji są zbyt "sztywne" dla układów sprzężonych, gdzie niewielkie błędy GNSS mogą powodować "walkę" dronów między sobą, prowadząc do nasycenia silników lub uszkodzeń strukturalnych.
*   **Sterowanie Impedancyjne (Impedance Control):** Wysoka sztywność, opór wobec sił zewnętrznych w celu utrzymania pozycji.
*   **Sterowanie Admitancyjne (Admittance Control):** Niska sztywność, jednostka "podporządkowuje się" siłom zewnętrznym i reaguje na nie ruchem.
System wykorzystuje **Sterowanie Admitancyjne**, modelując każdego drona jako wirtualny układ masa-sprężyna-tłumik. Naśladuje to biologiczną koordynację (np. mrówek transportujących pożywienie), gdzie jednostki dostosowują swój ruch na podstawie odczuwanych sił.

### 2.3 Optymalizacja: Sterowanie Predykcyjne (MPC)
Aby osiągnąć stabilny lot, każda jednostka uruchamia nieliniowy solver MPC (ACADO/OSQP) na komputerze pokładowym z częstotliwością ponad 50 Hz.
**Ograniczenia MPC:**
1.  **Limity Elementów Wykonawczych:** Silniki nie mogą przekroczyć maksymalnego ciągu.
2.  **Naciąg Liny:** $T > 0$ (Liny nigdy nie mogą stać się luźne, aby uniknąć gwałtownych szarpnięć).
3.  **Unikanie Kolizji:** Utrzymanie minimalnej bezpiecznej odległości między jednostkami.

## 3. Architektura Sprzętowa

### 3.1 Faza 1: Skalowany Prototyp ("Mikro" Rój)
PoC w mniejszej skali wykorzystująca rój 3-4 quadrocopterów do walidacji sterowania z ładunkiem 2-5 kg.

*   **Rama:** **Holybro X500 V2**. Ramiona z włókna węglowego zapewniają wysoką sztywność (kluczową dla eliminacji szumów IMU w pętlach sterowania).
*   **Jednostka Obliczeniowa:** **NVIDIA Jetson Orin Nano (8GB)**. Obsługuje MPC w czasie rzeczywistym, VIO oraz sieć Zenoh (wydajność AI 40 TOPS).
*   **Kontroler Lotu:** **Holybro Pixhawk 6C**. Procesor STM32H7, potrójnie redundantne jednostki IMU, izolacja wibracji.
*   **Czujniki:** 
    *   **GNSS:** **CubePilot Here 4**. Wielopasmowy RTK zapewniający centymetrową precyzję (DroneCAN).
    *   **Wizja:** **Intel RealSense D435i** lub **Luxonis OAK-D Pro**. Nawigacja wizyjno-inercyjna (VIO) dla środowisk bez sygnału GPS.
*   **Zasilanie:** **Matek BEC 12S-PRO**. Wejście 9-55V, wyjście 5A. Chroni Jetsona przed skokami napięcia podczas hamowania silników.

**Zestawienie Materiałowe Fazy 1 (Szacowany koszt jednostkowy: €2,490)**
| Komponent | Specyfikacja | Koszt |
| :--- | :--- | :--- |
| Rama | Holybro X500 V2 ARF | €380 |
| Kontroler Lotu | Pixhawk 6C | €215 |
| GNSS | Here 4 RTK | €285 |
| Komputer Pokładowy | Jetson Orin Nano 8GB | €450 |
| Kamera Głębi | RealSense D435i | €415 |
| Telemetria | SiYi MK15 | €550 |
| Inne | BEC, Akumulatory, Mocowania | €195 |

### 3.2 Faza 2: Pełnowymiarowy Transport Ciężki
Cel: całkowity udźwig 115 kg (80 kg człowiek + 10 kg nosze + 5 kg osprzęt + 20% marginesu bezpieczeństwa).

*   **Napęd:** **T-Motor U15 II KV80** (~36 kg ciągu na silnik) lub **Hobbywing XRotor X9 Plus** (26.5 kg ciągu na oś).
*   **Konfiguracja:** **Oktocopter (X8) Współosiowy**. Kluczowy dla redundancji; umożliwia bezpieczne lądowanie w przypadku awarii jednego silnika.
*   **Rama:** **Foxtech Gaia 160MP** (zmodyfikowana) lub niestandardowa rama z włókna węglowego CNC (rury 40 mm).
*   **Zasilanie:** **LiPo (Tattu Pro 12S 22000mAh)**. Wymagane ze względu na wysokie prądy rozładowania (25C+) niezbędne do generowania momentu stabilizującego. Ogniwa Li-Ion nie są zalecane ze względu na spadki napięcia pod obciążeniem.
*   **Osprzęt:** Uwięzi Dyneema (UHMWPE) ze zintegrowanymi amortyzatorami i serwo-mechanicznymi hakami bezpieczeństwa.

## 4. Stos Oprogramowania

### 4.1 Szkielet Systemu: ROS 2 Humble
Wykorzystuje zdecentralizowaną komunikację Peer-to-Peer. **Zarządzanie Cyklem Życia (Lifecycle Management)** zapewnia, że wszystkie kontrole bezpieczeństwa (fiks GPS, stan czujników) są aktywne przed uzbrojeniem silników.

### 4.2 Logika Sterowania: Rust
Obowiązkowy dla pętli sterowania krytycznych dla bezpieczeństwa, aby wyeliminować błędy pamięci i wyścigi (race conditions).
*   **MAVSDK-Rust:** Bezpieczny interfejs do sterowania offboard w PX4.
*   **ros2_rust:** Biblioteki klienckie dla węzłów ROS 2.

### 4.3 Komunikacja: Eclipse Zenoh
Zoptymalizowany dla sieci kratowych (mesh) nad niestabilnymi łączami bezprzewodowymi.
*   **Wydajność Wykrywania:** Redukuje ruch związany z wykrywaniem węzłów o 99% w porównaniu do DDS, unikając "sztormów odkrywania" (discovery storms) typowych dla protokołów multicastowych.
*   **Obsługa WAN:** Natywne trasowanie dla dalekosiężnej łączności ze Stacją Naziemną (GCS), również przez NAT.
*   **Mostkowanie:** `zenoh-bridge-ros2dds` do łączenia lokalnych tematów ROS 2 z globalną siecią roju, co pozwala dronom widzieć się nawzajem jako zdalne węzły przy zachowaniu wydajności transportu.

### 4.4 Konsensus i Koordynacja
*   **Konsensus Raft:** Implementacja w języku Rust zapewniająca niezawodne uzgadnianie stanu w roju (np. "Cel Potwierdzony", "Wybór Lidera").
*   **Podział Obszaru:** Dynamiczna tesselacja Woronoja lub licytacja sektorów na podstawie poziomu baterii i bliskości.
*   **Model Boids:** Zachowania stadne podczas przelotu (Separacja, Wyrównanie, Spójność).

## 5. Interfejsy Użytkownika i Interakcja

### 5.1 React GCS (Centrum Dowodzenia)
Stacjonarna Stacja Naziemna dla przeglądu taktycznego.
*   **Wizualizacja 3D:** Śledzenie dronów w czasie rzeczywistym i ścieżki misji przy użyciu CesiumJS/Leaflet.
*   **Stan Roju:** Siatka telemetrii, monitorowanie stanu baterii i siły sygnału (RSSI).
*   **Strumień Wideo:** Siatka wideo o niskich opóźnieniach dla świadomości sytuacyjnej.

### 5.2 Flutter Terminale Taktyczne
Mobilny interfejs dla ratowników terenowych na wzmocnionych tabletach.
*   **Flutter Rust Bridge:** Wysokowydajny interfejs UI wykorzystujący Rust do obsługi warstwy komunikacyjnej Zenoh.
*   **Tryb "Follow Me":** Koordynacja taktyczna, w której drony śledzą pozycję ratownika.

## 6. Interfejs Ładunku i Bezpieczeństwo Pacjenta

*   **Nosze:** Standardowe **NATO Litter (Stanag 2040)**.
*   **Rama Rozporowa:** Niestandardowa rama zapewniająca odpowiednią geometrię podnoszenia i odstęp od ciała pacjenta.
*   **Protokół Awaryjny:** Natychmiastowe odpięcie uwięzi uszkodzonej jednostki; pozostałe jednostki wykonują kontrolowane lądowanie awaryjne.

## 6. Zastosowania Morskie SAR

*   **Odporność Środowiskowa:** Norma IP67 i ochrona przed korozją solną (powłoki przemysłowe T-Motor).
*   **Dynamiczne Podejmowanie:** Kompensacja kołysania (heave compensation) przy lądowaniu na ruchomym pokładzie statku przy użyciu VIO i śledzenia znaczników AprilTag.
*   **Wzorce Poszukiwań:** Równoległe przeszukiwanie pasowe i wzorce "creeping line" zoptymalizowane pod kątem dryfu morskiego.

## 7. Protokoły Bezpieczeństwa i Zgodność Regulacyjna (SORA)

Projekt postępuje zgodnie z metodologią **SORA (Specific Operations Risk Assessment)** (EASA/ULC).
*   **Poziom SAIL:** Celowany **SAIL V/VI** (Wysokie ryzyko, granica kategorii "Certified").
*   **Redundancja:** Konfiguracja silników X8, potrójne IMU, niezależny System Zakończenia Lotu (FTS).
*   **Zapewnienie Jakości Oprogramowania:** Kod źródłowy w Rust projektowany zgodnie ze standardami zbliżonymi do DO-178C.
*   **Interakcja Człowiek-Rój:** Polecenia strategiczne (np. "Ewakuuj Cel") przez GCS oparty na **React** lub tablety taktyczne oparte na **Flutter** (używając `flutter_rust_bridge`).

### 7.1 Macierz Ryzyka i Mitygacja
| Ryzyko | Prawdopodobieństwo | Wpływ | Strategia Mitygacji |
| :--- | :--- | :--- | :--- |
| **Utrata Łączności** | Średnie | Wysoki | Autonomiczne procedury RTH w PX4; retransmisja przez sieć Zenoh Mesh. |
| **Awaria Napędu** | Niskie | Krytyczny | Redundancja koaksjalna X8; spadochron balistyczny dla jednostek Heavy Lift. |
| **Błąd Oprogramowania** | Wysokie (Dev) | Średny | Walidacja SITL przed lotem; Rust dla bezpieczeństwa pamięci. |
| **Odmowa Regulacyjna** | Średnie | Wysoki | Wczesny dialog z ULC/EASA; profesjonalne doradztwo SORA. |

## 8. Podsumowanie Finansowe i Harmonogram

*   **Całkowity Budżet:** ~€270,000 (obejmuje B+R, sprzęt i doradztwo regulacyjne).
*   **Czas trwania:** 18 miesięcy od etapu PoC Fazy 1 do certyfikacji Fazy 2.
*   **Obecny Status:** Oprogramowanie Fazy 1 zwalidowane w SITL; rozpoczęto proces zakupowy sprzętu.
