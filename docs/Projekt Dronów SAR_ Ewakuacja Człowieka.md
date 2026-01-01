# **PROJEKT SYSTEMU AUTONOMICZNEGO ROJU DRONÓW SAR DO EWAKUACJI OSÓB: STUDIUM TECHNICZNE, ARCHITEKTURA I PLAN WDROŻENIA**

## **1\. Wstęp i Kontekst Operacyjny Nowoczesnych Systemów Ratowniczych**

Współczesne operacje poszukiwawczo-ratownicze (SAR – Search and Rescue) stoją w obliczu rosnących wymagań dotyczących szybkości reakcji, precyzji działania oraz minimalizacji ryzyka dla personelu ratowniczego. Tradycyjne metody, opierające się na pieszych zespołach przeszukujących teren, psach tropiących oraz załogowym lotnictwie śmigłowcowym, choć wciąż niezastąpione w wielu aspektach, napotykają na bariery wynikające z fizjologii człowieka, warunków atmosferycznych oraz kosztów operacyjnych.1 Wprowadzenie bezzałogowych statków powietrznych (UAV) do arsenału służb ratowniczych stanowiło pierwszy krok w kierunku cyfryzacji pola walki o życie, jednakże model „jeden pilot – jeden dron” szybko okazał się niewystarczający w obliczu rozległych obszarów poszukiwań i złożoności zadań ewakuacyjnych.2

Niniejszy raport techniczny przedstawia kompleksową koncepcję, architekturę oraz plan wdrożenia zaawansowanego systemu SAR opartego na roju dronów (SDS – Swarm Drone System). Projekt zakłada wykorzystanie kooperacji wielu autonomicznych jednostek do realizacji dwóch kluczowych celów: szybkiej lokalizacji osoby poszkodowanej przy użyciu małych, zwrotnych dronów zwiadowczych (Faza 1 – Proof of Concept) oraz jej fizycznej ewakuacji przy użyciu ciężkich platform transportowych klasy Heavy Lift (Faza 2 – Wdrożenie). Analiza obejmuje szczegółowy dobór komponentów sprzętowych kompatybilnych z ekosystemem Nvidia Jetson, architekturę oprogramowania hybrydowego łączącą wydajność języka Rust z elastycznością Pythona oraz nowoczesne interfejsy operacyjne oparte na React i Flutter. Całość osadzona jest w realiach prawnych i certyfikacyjnych Unii Europejskiej oraz Polski, ze szczególnym uwzględnieniem metodologii SORA.

### **1.1. Ewolucja Paradygmatu: Od Teleoperacji do Autonomicznego Roju**

Dotychczasowe wykorzystanie dronów w SAR ograniczało się zazwyczaj do roli „latającej lornetki”, gdzie operator manualnie sterował statkiem i interpretował obraz wideo. Takie podejście generuje znaczne obciążenie poznawcze (cognitive load) dla operatora, co w warunkach stresu prowadzi do błędów i przeoczeń.1 Przejście na systemy rojowe zmienia ten paradygmat. Rój, definiowany jako grupa dronów działająca kooperacyjnie pod nadzorem jednego systemu, pozwala na automatyzację procesu decyzyjnego. Zamiast sterować każdym wirnikiem, dowódca akcji wydaje polecenia wysokiego poziomu, np. „przeszukaj sektor A” lub „ewakuuj cel B”.3

Technologia SDS (Swarm Drone System) czerpie inspiracje z biologicznych modeli zachowań stadnych (Boids model), takich jak separacja (unikanie kolizji z sąsiadami), wyrównanie (dostosowanie prędkości do grupy) i spójność (utrzymywanie się w grupie). Implementacja tych zasad w algorytmach sterowania pozwala na uzyskanie systemu odpornego na awarie pojedynczych jednostek – utrata jednego drona zwiadowczego nie przerywa misji, gdyż rój automatycznie rekonfiguruje szyk i redystrybuuje zadania.4

### **1.2. Wyzwanie Ewakuacji: Udźwig i Dynamika Transportu**

O ile zwiad przy użyciu roju jest technologią wchodzącą w fazę dojrzałości, o tyle ewakuacja człowieka (ładunek rzędu 80–100 kg wliczając nosze i sprzęt medyczny) stanowi ogromne wyzwanie inżynieryjne. W literaturze naukowej rozważa się dwa podejścia: transport kooperacyjny, gdzie wiele mniejszych dronów wspólnie unosi ładunek na linach 6, oraz transport jednolity przy użyciu dedykowanego drona klasy Heavy Lift.8

Transport kooperacyjny (Cooperative Payload Transport) oferuje teoretyczną elastyczność i skalowalność, jednak wiąże się ze skrajnie skomplikowaną dynamiką układu wielowahadłowego. Oscylacje ładunku na linach, sprzężenia zwrotne między dronami oraz konieczność synchronizacji ciągu z milisekundową precyzją sprawiają, że jest to rozwiązanie wciąż eksperymentalne i ryzykowne w kontekście transportu ludzi.10 Z tego względu, w niniejszym projekcie przyjęto strategię hybrydową: Faza 1 skupia się na inteligencji roju i zwiadzie, natomiast Faza 2 wprowadza potężne, wyspecjalizowane jednostki Heavy Lift do samej ekstrakcji, wspierane sensorycznie przez rój mniejszych jednostek.

## **2\. Architektura Oprogramowania: Fundament Niezawodności**

W systemach krytycznych dla życia, takich jak SAR, oprogramowanie nie może być jedynie funkcjonalne – musi być deterministyczne, bezpieczne i odporne na błędy. Zaproponowana architektura odchodzi od monolitycznych rozwiązań na rzecz rozproszonego systemu mikroserwisów, wykorzystując specyficzne zalety różnych języków programowania i nowoczesnych protokołów komunikacyjnych.

### **2.1. System Operacyjny Robota: ROS 2 jako Kręgosłup Systemu**

Jako środowisko bazowe wybrano **ROS 2 (Robot Operating System 2\)**, w dystrybucji Humble Hawksbill (LTS) lub nowszej Jazzy Jalisco. Wybór ROS 2 zamiast ROS 1 jest podyktowany fundamentalnymi różnicami w architekturze komunikacyjnej. ROS 1 polegał na centralnym węźle roscore, którego awaria paraliżowała cały system – ryzyko nieakceptowalne w operacjach polowych. ROS 2 jest w pełni zdecentralizowany, co oznacza, że drony mogą komunikować się bezpośrednio (Peer-to-Peer), a system nie posiada pojedynczego punktu awarii (Single Point of Failure).5

Kluczowym aspektem ROS 2 jest obsługa mechanizmów Quality of Service (QoS). Pozwala to na precyzyjne definiowanie niezawodności transmisji dla różnych typów danych:

* **Dane telemetryczne i sterujące:** QoS ustawione na *Reliable* (gwarancja dostarczenia), co jest krytyczne dla komend ruchu.  
* **Strumienie wideo i LIDAR:** QoS ustawione na *Best Effort* (najlepsza próba), gdzie zgubienie pojedynczej klatki jest akceptowalne na rzecz zachowania niskich opóźnień.4

### **2.2. Poliglotyczna Strategia Implementacji: Rola Rusta i Pythona**

#### **2.2.1. Rust: Bezpieczeństwo i Wydajność w Warstwie Krytycznej**

Do implementacji kluczowych modułów systemu (sterowanie lotem, fuzja sensorów, obsługa komunikacji roju) rekomenduje się język **Rust**. Jest to nowoczesny język systemowy, który gwarantuje bezpieczeństwo pamięci na etapie kompilacji, eliminując całe klasy błędów takich jak wycieki pamięci, odwołania do zwolnionych wskaźników czy wyścigi danych (data races) w wątkach.13

W kontekście robotyki roju, Rust oferuje:

* **Determinizm:** Brak Garbage Collectora (obecnego w Pythonie czy Javie) sprawia, że czasy wykonania kodu są przewidywalne, co jest niezbędne w pętlach sterowania czasu rzeczywistego (Real-Time Control Loops).  
* **Ekosystem:** Biblioteki takie jak r2r, ros2\_rust czy zenoh (napisany natywnie w Rust) umożliwiają budowę wydajnych węzłów ROS 2 o wydajności porównywalnej z C++, ale przy znacznie wyższym poziomie bezpieczeństwa kodu.13

#### **2.2.2. Python: Elastyczność w Warstwie Aplikacyjnej i AI**

Python pozostaje niezastąpiony w obszarach szybkiego prototypowania oraz integracji z modelami uczenia maszynowego. W projektowanym systemie Python będzie wykorzystywany głównie na komputerach pokładowych Nvidia Jetson do:

* **Computer Vision:** Obsługa frameworków PyTorch i TensorFlow do detekcji ludzi na obrazach z kamer termowizyjnych i wizyjnych.  
* Logika wysokopoziomowa: Skrypty zarządzające sekwencjami misji w fazie eksperymentalnej, gdzie częste zmiany w kodzie są normą.  
  Należy jednak ściśle izolować moduły pythonowe od pętli sterowania silnikami, aby ewentualne opóźnienia interpretera nie wpłynęły na stabilność lotu.

### **2.3. Rewolucja w Komunikacji: Zenoh zamiast DDS**

Standardowym middleware dla ROS 2 jest DDS (Data Distribution Service). Jednakże, DDS został zaprojektowany z myślą o stabilnych sieciach przewodowych (LAN). W środowiskach bezprzewodowych (WiFi, LTE), typowych dla rojów dronów, mechanizm odkrywania węzłów w DDS (Discovery Protocol), oparty na ciągłym rozsyłaniu pakietów multicast, generuje tzw. „sztormy odkrywania” (discovery storms), które potrafią całkowicie zapchać łącze radiowe przy większej liczbie jednostek.16

Jako rozwiązanie tego problemu, raport rekomenduje wdrożenie protokołu **Eclipse Zenoh**.

* **Redukcja narzutu:** Zenoh redukuje ruch discovery nawet o 99% w porównaniu do DDS, co jest kluczowe przy ograniczonej przepustowości łączy radiowych.17  
* **Architektura:** Zenoh działa w modelu Pub/Sub/Query i jest zoptymalizowany pod kątem sieci o wysokiej latencji i niskiej przepustowości.  
* **Integracja:** Zastosowanie zenoh-bridge-dds pozwala na transparentne mostkowanie lokalnych systemów ROS 2 na każdym dronie z siecią roju. Dron „widzi” inne drony jako zdalne węzły, ale fizyczna transmisja danych odbywa się przez wydajny protokół Zenoh, a nie ciężki DDS.19

### **2.4. Nowoczesne Interfejsy Operatora: React i Flutter**

#### **2.4.1. React: Stacjonarne Centrum Dowodzenia (GCS)**

Dla stanowiska dowodzenia w bazie (pojazd sztabowy/namiot) przewidziano aplikację webową opartą na bibliotece **React**.

* **Wizualizacja:** Wykorzystanie komponentów takich jak react-leaflet lub CesiumJS do wyświetlania pozycji dronów na trójwymiarowej mapie terenu w czasie rzeczywistym.  
* **Komunikacja z Rojem:** Aplikacja łączy się z siecią Zenoh poprzez WebSocket lub REST API, umożliwiając podgląd parametrów telemetrycznych, stanu baterii i strumieni wideo z minimalnym opóźnieniem.21

#### **2.4.2. Flutter: Mobilne Terminale Taktyczne**

Ratownicy działający w terenie, często w trudnych warunkach, potrzebują lekkiego i niezawodnego interfejsu na tabletach lub smartfonach. Tutaj wybór pada na framework **Flutter**.

* **Wydajność natywna:** Flutter kompiluje się do kodu maszynowego (ARM), co zapewnia płynność działania interfejsu (60/120 FPS) nawet na urządzeniach mobilnych średniej klasy.  
* **Flutter Rust Bridge:** Unikalnym atutem jest możliwość wykorzystania biblioteki flutter\_rust\_bridge. Pozwala ona na napisanie całej warstwy komunikacyjnej i logicznej aplikacji w języku Rust (korzystając z klienta Zenoh), a Fluttera używać jedynie jako warstwy prezentacji (UI). Gwarantuje to, że aplikacja mobilna będzie miała tę samą niezawodność komunikacji co oprogramowanie pokładowe dronów.23

## **3\. Faza 1: Proof of Concept (PoC) – Budowa Roju Zwiadowczego**

Celem Fazy 1 jest opracowanie i przetestowanie algorytmów koordynacji roju, systemów detekcji oraz architektury komunikacyjnej na mniejszej, bezpieczniejszej i tańszej platformie sprzętowej. Drony te pełnić będą później funkcję zwiadowców i sensorów wspierających dla ciężkiego drona transportowego.

### **3.1. Szczegółowa Specyfikacja Sprzętowa Platformy PoC**

Wybór komponentów dla fazy PoC podyktowany jest koniecznością zapewnienia dużej mocy obliczeniowej dla algorytmów AI (na pokładzie) przy zachowaniu umiarkowanych kosztów i bezpieczeństwa operacji.

| Podsystem | Komponent | Szczegóły Techniczne i Uzasadnienie | Cena Jedn. (Szacunek) |
| :---- | :---- | :---- | :---- |
| **Komputer Pokładowy** | **Nvidia Jetson Orin Nano (8GB)** | Serce systemu. Oferuje wydajność AI na poziomie **40 TOPS**, co jest skokowym wzrostem względem starszych modeli (Nano/TX2). Posiada 6-rdzeniowy procesor ARM Cortex-A78AE, co pozwala na równoległe uruchamianie węzłów ROS 2 i inferencję sieci neuronowych. Wersja 8GB RAM jest niezbędna do obsługi dużych modeli wizyjnych.25 | \~1 400 PLN |
| **Rama i Napęd** | **Holybro X500 V2 ARF Kit** | Standard badawczy. Rama o przekątnej 500mm z włókna węglowego. Zestaw zawiera silniki 2216 KV880 i regulatory ESC 20A. Konstrukcja zapewnia łatwy montaż Jetsona i peryferiów bez konieczności druku 3D niestandardowych elementów.27 | \~1 500 PLN |
| **Autopilot** | **Pixhawk 6C** | Kontroler lotu oparty na mikrokontrolerze STM32H743 (480MHz). Posiada redundantne czujniki inercyjne (IMU) izolowane od wibracji. Komunikuje się z Jetsonem przez szybki interfejs szeregowy, realizując niskopoziomową stabilizację lotu.29 | \~800 PLN |
| **System Wizyjny (Głębia)** | **Luxonis OAK-D Pro** | Kamera stereo z aktywnym oświetlaczem IR (laser dot projector). Kluczowa przewaga nad Intel RealSense w SAR: posiada wbudowany procesor VPU Myriad X, który wykonuje obliczenia mapy głębi i detekcji obiektów, nie obciążając głównego procesora Jetsona. Laser IR pozwala na działanie w całkowitej ciemności i na powierzchniach o niskiej teksturze (śnieg, trawa).31 | \~1 300 PLN |
| **Pozycjonowanie** | **Holybro Here 3+ (RTK GNSS)** | Moduł GPS oparty na chipie u-blox M8P/F9P. Obsługa RTK (Real-Time Kinematic) zapewnia precyzję pozycjonowania rzędu 2-3 cm, co jest absolutnie krytyczne dla lotów w formacji roju, aby uniknąć kolizji między dronami.33 | \~700 PLN |
| **Komunikacja Danych** | **Moduł WiFi 6 (Intel AX210)** | Do celów deweloperskich i testów na bliskim zasięgu. Zapewnia wysoką przepustowość dla debugowania i przesyłu wideo. | \~100 PLN |
| **Zasilanie** | **LiPo 4S 5000mAh 60C** | Standardowy pakiet zapewniający ok. 15-20 minut lotu z pełnym obciążeniem obliczeniowym. | \~250 PLN |
| **Suma (1 dron)** |  |  | **\~6 050 PLN** |

**Rekomendacja:** Zbudowanie floty składającej się z minimum 4 jednostek tego typu (3 operacyjne \+ 1 zapasowa/rozwojowa).

### **3.2. Architektura Systemu Pokładowego (On-Board Architecture)**

Integracja systemów na pokładzie drona PoC jest wielowarstwowa:

1. **Warstwa Real-Time (Pixhawk 6C):** Odpowiada za utrzymanie równowagi, sterowanie silnikami i estymację stanu (EKF). Działa na systemie operacyjnym czasu rzeczywistego (NuttX) z firmwarem PX4 lub ArduPilot.  
2. **Warstwa Pośrednia (MAVLink Router):** Fizyczne połączenie UART między Pixhawkiem a Jetsonem.  
3. **Warstwa Wysokiego Poziomu (Jetson Orin Nano):**  
   * **Micro-ROS Agent:** Mostkuje dane z Pixhawka (przez MAVLink lub XRCE-DDS) do środowiska ROS 2 na Jetsonie.  
   * **Perception Node (Python):** Odbiera obraz z kamery OAK-D. Wykonuje detekcję (np. model YOLOv8-nano wytrenowany na sygnatury cieplne/wizualne ludzi). Publikuje koordynaty celu.  
   * **Swarm Control Node (Rust):** Implementuje logikę roju. Odbiera pozycje innych dronów (przez Zenoh), oblicza wektory prędkości (unikanie kolizji \+ dążenie do celu) i wysyła komendy sterujące do Pixhawka.35

### **3.3. Algorytmy Roju i Koordynacja**

W fazie PoC zaimplementowane zostaną algorytmy rozproszone, niewymagające centralnego serwera sterującego ruchem każdego drona (co zwiększa odporność systemu).

* **Konsensus (Raft):** Do uzgodnienia wspólnych decyzji (np. "znalazłem cel, przerywamy poszukiwania, formujemy krąg") wykorzystany zostanie algorytm Raft. Implementacja w Rust (raft-rs) zapewnia niezawodność procesu uzgadniania stanu między dronami w warunkach potencjalnych utrat pakietów.36  
* **Podział Obszaru:** Algorytm dynamicznej tesselacji Woronoja lub proste przydzielanie sektorów. Każdy dron "licytuje" sektory do przeszukania na podstawie swojego poziomu baterii i pozycji.

## **4\. Faza 2: Platforma Heavy Lift i System Ewakuacji**

Przejście do Fazy 2 oznacza wdrożenie systemu zdolnego do fizycznego podniesienia osoby poszkodowanej. Wymaga to dronów o masie startowej (MTOW) przekraczającej 100 kg.

### **4.1. Analiza Fizyki Transportu: Pojedynczy Dron vs Rój Nośny**

Zasadniczym dylematem jest wybór metody transportu.

* **Transport Kooperacyjny (Rój Nośny):** Koncepcja, w której 4 mniejsze drony chwytają narożniki noszy. Zalety: łatwiejszy transport dronów na miejsce akcji. Wady: Ekstremalna trudność sterowania. Układ taki tworzy złożone wahadło wieloczłonowe. Nierównomierny wiatr lub awaria jednego drona może doprowadzić do destabilizacji całego układu i katastrofy.6 Wymagałoby to zaawansowanych algorytmów sterowania predykcyjnego (MPC) i idealnej synchronizacji łączy, co w warunkach polowych jest ryzykowne.  
* **Pojedynczy Heavy Lift:** Jeden duży dron (układ oktocoptera) o udźwigu 80-100 kg. Jest to rozwiązanie sprawdzone w logistyce przemysłowej i wojskowej.8 Dynamika jest prostsza (ładunek podwieszony pod środkiem ciężkości), a redundancja silników (X8) zapewnia bezpieczeństwo.

**Decyzja Projektowa:** Ze względów bezpieczeństwa i certyfikacji, projekt przyjmuje model **jednego drona Heavy Lift** do transportu, wspieranego przez rój dronów z Fazy 1 (oświetlenie, retransmisja sygnału, zwiad).

### **4.2. Specyfikacja Techniczna Drona Ewakuacyjnego (Heavy Lift)**

| Podsystem | Komponent / Model | Charakterystyka Techniczna | Szacunkowy Koszt |
| :---- | :---- | :---- | :---- |
| **Rama Nośna** | **Foxtech Gaia 160MP** lub Custom Carbon X8 | Konstrukcja o rozpiętości ramion ok. 1600mm. Układ koaksjalny X8 (8 silników na 4 ramionach) zapewnia kompaktowość przy zachowaniu redundancji. Włókno węglowe gwarantuje sztywność niezbędną przy tak dużych obciążeniach.38 | \~22 000 PLN |
| **Zespół Napędowy** | **T-Motor U13 II KV130** (x8) | Silniki klasy "Heavy Lift". Każdy generuje ciąg max ok. 24 kg przy zasilaniu 12S. Łączny ciąg roju: \~192 kg. Przy masie startowej (dron+ładunek) rzędu 100-110 kg, daje to bezpieczny stosunek ciągu do wagi bliski 2:1, niezbędny do manewrowania w wietrze.40 | \~14 000 PLN |
| **Śmigła** | **T-Motor 30x10.5 CF** | 30-calowe śmigła węglowe. Ich duża inercja zapewnia stabilność, ale wymaga precyzyjnego sterowania ESC. | \~2 500 PLN |
| **Regulatory (ESC)** | **T-Motor Flame 100A HV** | Chłodzone powietrzem, uszczelnione (IP55) regulatory wysokiego napięcia. Muszą obsługiwać prądy szczytowe do 100A i napięcie 50V.40 | \~3 500 PLN |
| **Zasilanie (Baterie)** | **Tattu Plus 1.0 22000mAh 12S** (x6) | Pakiety o napięciu 44.4V. Wymagane połączenie równoległe min. 4-6 sztuk, aby zapewnić wydajność prądową i czas lotu rzędu 15-20 min z ładunkiem. System BMS monitoruje każdą celę.42 | \~25 000 PLN |
| **System Ładowania** | **SkyRC PC3000** | Przemysłowa stacja ładowania o mocy 3000W. Niezbędna do szybkiego (poniżej 1h) ładowania ogromnych pakietów 12S w terenie (z agregatu prądotwórczego).44 | \~3 200 PLN |
| **Autopilot** | **Pixhawk 6X / Cube Orange+** | Wersja przemysłowa z potrójną redundancją czujników i systemem podgrzewania IMU (praca w niskich temperaturach). | \~2 000 PLN |
| **Łączność Dalekiego Zasięgu** | **SiYi HM30 / MK15** | System transmisji wideo i telemetrii o zasięgu do 30 km. Działa w paśmie 5GHz/2.4GHz, oferując niskie opóźnienia (latency \~150ms) dla obrazu 1080p, co jest kluczowe dla operatora.46 | \~2 800 PLN |
| **Suma (Platforma)** |  |  | **\~75 000 PLN** |

### **4.3. Mechanizm Podejmowania (Payload Mechanism)**

Dron zostanie wyposażony w system wciągarki lub sztywnego zaczepu (cargo hook) z systemem ważącym (load cell).

* **Procedura:** Dron zawisa nad poszkodowanym na wysokości ok. 5-10 metrów (powyżej koron drzew). Opuszcza linę z uprzężą/noszami. Ratownik na ziemi (lub sam poszkodowany, jeśli przytomny) wpina się w system.  
* **Bezpieczeństwo:** System zaczepu musi posiadać pirotechniczny lub elektromagnetyczny zrzut awaryjny (Emergency Release). Jeśli ładunek zaczepi się o przeszkodę lub wpadnie w niekontrolowane oscylacje zagrażające dronowi, pilot musi mieć możliwość natychmiastowego odrzucenia ładunku, aby ocalić platformę (i uniknąć upadku 100-kilogramowej maszyny na ludzi).

## **5\. Szczegółowa Architektura Oprogramowania i Integracja**

### **5.1. Hybrydowy Model Danych i Komunikacji**

Architektura oprogramowania musi sprostać wymaganiom czasu rzeczywistego oraz rozproszenia geograficznego.

#### **Tabela: Stos Technologiczny (Tech Stack)**

| Warstwa | Technologia | Rola w Systemie | Uzasadnienie |
| :---- | :---- | :---- | :---- |
| **Firmware Low-Level** | **PX4 / ArduPilot** | Stabilizacja lotu, obsługa silników | Standard przemysłowy, certyfikowalność. |
| **Middleware** | **Eclipse Zenoh** | Komunikacja Rój-Rój i Rój-Ziemia | Minimalny narzut discovery, routing przez NAT, oszczędność pasma.17 |
| **Core Logic (Jetson)** | **Rust** (ros2\_rust) | Maszyna stanów, fuzja sensorów | Bezpieczeństwo pamięci, wydajność.14 |
| **AI / Vision** | **Python** (PyTorch) | Detekcja obiektów (YOLO) | Łatwość użycia modeli ML, ekosystem Nvidia JetPack. |
| **Baza Danych** | **InfluxDB** (przez Zenoh) | Rejestracja telemetrii (Czarna Skrzynka) | Wydajność w zapisie szeregów czasowych. |
| **Frontend Web** | **React \+ TypeScript** | Dashboard Dowódcy | Komponentowość, ekosystem wizualizacji (Leaflet). |
| **Frontend Mobile** | **Flutter \+ Rust FFI** | Terminal Ratownika | Wydajność, cross-platform, logika w Rust.23 |

### **5.2. Implementacja Węzłów Systemowych**

A. Węzeł AI (Python \- Jetson):  
Działa w kontenerze Docker. Wykorzystuje rclpy do komunikacji z ROS 2\.

1. Pobiera klatkę z kamery OAK-D (topik /oak/rgb/image\_raw).  
2. Przepuszcza przez model YOLOv8 (zoptymalizowany TensorRT).  
3. Jeśli wykryto człowieka z pewnością \> 80%:  
   * Oblicza pozycję 3D względem drona (korzystając z mapy głębi).  
   * Transformuje pozycję do układu globalnego (GPS).  
   * Publikuje wiadomość PersonDetection na topiku /perception/human\_found.

B. Węzeł Roju (Rust \- Jetson):  
Krytyczny komponent napisany w Rust.

1. Subskrybuje /perception/human\_found.  
2. Po otrzymaniu zgłoszenia, inicjuje proces **Raft Consensus** z innymi dronami, aby potwierdzić znalezisko i wyznaczyć drona, który ma najlepszy widok (Leader Election).  
3. Lider Roju wysyła komunikat do Stacji Naziemnej (przez Zenoh) i przełącza pozostałe drony w tryb "Orbitowania" wokół celu, tworząc sieć przekaźnikową dla wideo.

C. Mostek Zenoh (Zenoh-Bridge-DDS):  
Jest to proces działający w tle. Konfiguracja JSON definiuje, które topiki ROS 2 są widoczne na zewnątrz.

* *Allow-list:* /drone\_id/pose, /drone\_id/battery, /mission/status.  
* Block-list: /camera/raw, /lidar/points (zbyt duże dane, blokowane by nie zapchać łącza).  
  Wideo przesyłane jest poza ROS 2, dedykowanym strumieniem RTP lub pluginem Zenoh do przesyłu dużych obiektów (Large Payload).

### **5.3. Interfejsy Użytkownika**

React Dashboard (Stacja Bazowa):  
Aplikacja wykorzystuje zenoh-javascript.

* **Mapa Taktyczna:** Wyświetla pozycje dronów, ich ścieżki (trails) oraz zaznaczone obszary poszukiwań (poligony).  
* **Panel Wideo:** Siatka (Grid) z podglądem z kamer wybranych dronów (niskiej jakości preview, wysokiej jakości na żądanie).  
* **Stan Roju:** Tabela z poziomem baterii, siłą sygnału RSSI i statusem GPS każdego drona.

Flutter App (Ratownik):  
Aplikacja na tablet przemysłowy.

* Wykorzystuje flutter\_rust\_bridge. Warstwa Rust obsługuje klienta Zenoh, dekoduje wiadomości Protobuf i przekazuje obiekty Dart do warstwy UI.  
* Funkcja "Follow Me": Dron transportowy może automatycznie podążać za sygnałem GPS z tabletu ratownika.

## **6\. Aspekty Prawne, Certyfikacja i Bezpieczeństwo (Polska/UE)**

Eksploatacja roju dronów, a zwłaszcza transport ludzi, podlega rygorystycznym regulacjom EASA wdrażanym w Polsce przez Urząd Lotnictwa Cywilnego (ULC).

### **6.1. Klasyfikacja Operacji**

Projekt wykracza poza kategorię Otwartą.

* **Faza 1 (Małe drony):** Może być realizowana w **Kategorii Szczególnej (Specific)** w ramach scenariuszy standardowych (STS) lub krajowych (NSTS), o ile loty odbywają się w zasięgu wzroku (VLOS) lub poza nim (BVLOS) nad terenami słabo zaludnionymi.  
* **Faza 2 (Heavy Lift):** Transport ładunków niebezpiecznych lub ludzi teoretycznie wpada w **Kategorię Certyfikowaną (Certified)**. Jednakże, operacje państwowe (SAR, Policja, Wojsko) są często wyłączone z ogólnych przepisów EASA na mocy art. 2 ust. 3 Rozporządzenia (UE) 2018/1139. W przypadku podmiotów cywilnych (np. GOPR/TOPR) wspierających służby, konieczne będzie uzyskanie Zezwolenia na Operację (Operational Authorisation) opartego na pełnej analizie SORA.47

### **6.2. Analiza Ryzyka SORA (Specific Operations Risk Assessment)**

Dla drona o rozpiętości 1.6m i masie \>25kg (Faza 2):

1. **Ground Risk Class (GRC):** Przy lotach nad terenem niezamieszkanym (góry, lasy) GRC może wynosić ok. 4-5. Jednak systemy mitygujące (spadochron ratunkowy) mogą obniżyć tę klasę o 1-2 punkty.  
2. **Air Risk Class (ARC):** Wymagana segregacja przestrzeni powietrznej. Zgłoszenie strefy TRA (Temporary Reserved Area) lub strefy D (Danger) w PAŻP na czas akcji.  
3. **Specific Assurance and Integrity Level (SAIL):** Wynikowa ocena SAIL (prawdopodobnie poziom III lub IV) narzuci wymogi techniczne, takie jak redundancja C2 (Command & Control) i niezależny system FTS (Flight Termination System).

### **6.3. Integracja z PansaUTM**

Każdy lot roju musi być koordynowany z Polską Agencją Żeglugi Powietrznej.

* Operator roju używa aplikacji **DroneTower** do zgłoszenia misji ("Check-in").  
* System roju powinien być zintegrowany z systemami **Remote ID**, nadając swoją pozycję w czasie rzeczywistym, aby inne statki powietrzne (np. śmigłowiec LPR) widziały drony w systemach antykolizyjnych.

## **7\. Plan Wdrożenia, Kosztorys i Analiza Ryzyka**

### **7.1. Harmonogram Realizacji (Roadmapa)**

* **Miesiące 1-3 (Inkubacja):**  
  * Zakup sprzętu Fazy 1\.  
  * Konfiguracja środowiska deweloperskiego (Docker, ROS 2, Zenoh).  
  * Implementacja podstawowych algorytmów roju w symulatorze Gazebo/Ignition.  
* **Miesiące 4-6 (PoC Roju):**  
  * Integracja software z hardware na dronach X500.  
  * Testy poligonowe: formacje, autonomiczne poszukiwanie, testy zasięgu łącza radiowego.  
  * Demonstracja lokalizacji celu (manekin).  
* **Miesiące 7-12 (Budowa Heavy Lift):**  
  * Zamówienie komponentów Fazy 2 (czas oczekiwania na silniki/ramy z Azji to często 4-8 tygodni).  
  * Montaż i testy naziemne (hamownia) układu napędowego.  
  * Integracja awioniki Pixhawk 6X i strojenie PID dla dużej masy.  
* **Miesiące 13-18 (Integracja Systemu):**  
  * Loty testowe Heavy Lift z balastem (dummy load).  
  * Integracja roju zwiadowczego z dronem transportowym (wspólna sieć Zenoh).  
  * Testy systemu zrzutu i podejmowania ładunku.  
  * Proces certyfikacji i dopuszczenia do lotów (dokumentacja SORA).

### **7.2. Szczegółowy Budżet Projektu (Szacunek Netto)**

Poniższy kosztorys obejmuje sprzęt i licencje, nie zawiera kosztów pracy zespołu R\&D.

| Kategoria | Pozycja | Ilość | Koszt Jedn. (PLN) | Suma (PLN) |
| :---- | :---- | :---- | :---- | :---- |
| **Sprzęt Faza 1** | Dron X500 \+ Jetson \+ OAK-D \+ Baterie | 4 | 6 500 | 26 000 |
| **Sprzęt Faza 2** | Platforma Heavy Lift (Silniki, Rama, ESC) | 1 | 75 000 | 75 000 |
|  | Zapasowy zestaw napędowy (ramię, silnik, śmigło) | 1 | 5 000 | 5 000 |
|  | Pakiety Tattu 22Ah 12S (na 3 loty) | 12 | 3 500 | 42 000 |
| **Stacja Naziemna** | Laptopy Rugged, Routery Zenoh, Maszty antenowe | 1 | 20 000 | 20 000 |
| **Łączność** | System SiYi HM30 / MK15 (Ground \+ Air units) | 2 | 3 000 | 6 000 |
| **Bezpieczeństwo** | Spadochrony balistyczne (dla Heavy Lift) | 1 | 8 000 | 8 000 |
| **Logistyka** | Skrzynie transportowe, Agregat prądotwórczy | 1 | 10 000 | 10 000 |
| **Rezerwa** | Nieprzewidziane wydatki (10%) | \- | \- | \~19 000 |
| **SUMA CAŁKOWITA** |  |  |  | **\~211 000 PLN** |

### **7.3. Macierz Ryzyka**

| Ryzyko | Prawdopodobieństwo | Wpływ | Strategia Mitygacji |
| :---- | :---- | :---- | :---- |
| **Utrata łączności z rojem** | Średnie | Wysokie | Autonomiczne procedury Return-to-Home na poziomie Pixhawka. Sieć Mesh (Zenoh) pozwalająca na retransmisję przez inne drony. |
| **Awaria napędu Heavy Lift** | Niskie | Krytyczne | Układ X8 (redundancja). Jeśli silnik padnie, dron ląduje awaryjnie. Spadochron balistyczny jako ostateczność. |
| **Błędy w oprogramowaniu roju** | Wysokie (w fazie dev) | Średnie | Rygorystyczne testy w symulatorze (SITL) przed każdym wgraniem kodu na drony. Użycie Rust do eliminacji błędów pamięci. |
| **Brak zgody ULC na loty** | Średnie | Wysokie | Wczesne rozpoczęcie dialogu z ULC. Przygotowanie profesjonalnej dokumentacji SORA przez certyfikowanych specjalistów. |

## **8\. Podsumowanie i Wnioski**

Opracowany plan techniczny i operacyjny wykazuje, że budowa autonomicznego roju dronów SAR zdolnego do ewakuacji osób jest przedsięwzięciem wykonalnym technologicznie przy obecnym stanie wiedzy. Kluczem do sukcesu jest odejście od eksperymentalnych metod transportu kooperacyjnego na rzecz sprawdzonej platformy Heavy Lift, przy jednoczesnym przeniesieniu ciężaru innowacji na warstwę oprogramowania (Rust, Zenoh, AI) i koordynacji roju zwiadowczego.

Taka konfiguracja – **Inteligencja Roju \+ Siła Giganta** – zapewnia optymalny balans między ryzykiem operacyjnym a efektywnością misji ratunkowej. System ten ma potencjał zrewolucjonizować ratownictwo górskie i wodne w Polsce, skracając czas dotarcia do poszkodowanego z godzin do minut.

#### **Works cited**

1. Drone Swarms to Support Search and Rescue Operations: Opportunities and Challenges \- Niels van Berkel, accessed on December 11, 2025, [https://nielsvanberkel.com/files/publications/culturalrobotics2023a.pdf](https://nielsvanberkel.com/files/publications/culturalrobotics2023a.pdf)  
2. Drone Swarms to Support Search and Rescue Operations: Opportunities and Challenges, accessed on December 11, 2025, [https://www.researchgate.net/publication/370695486\_Drone\_Swarms\_to\_Support\_Search\_and\_Rescue\_Operations\_Opportunities\_and\_Challenges](https://www.researchgate.net/publication/370695486_Drone_Swarms_to_Support_Search_and_Rescue_Operations_Opportunities_and_Challenges)  
3. Collaborative Control Technology and Mission Execution Capability of the Swarm Drone System \- Atlantis Press, accessed on December 11, 2025, [https://www.atlantis-press.com/article/126016686.pdf](https://www.atlantis-press.com/article/126016686.pdf)  
4. Intelligent Swarm: Concept, Design and Validation of Self-Organized UAVs Based on Leader–Followers Paradigm for Autonomous Mission Planning \- MDPI, accessed on December 11, 2025, [https://www.mdpi.com/2504-446X/8/10/575](https://www.mdpi.com/2504-446X/8/10/575)  
5. ROS2swarm \- A ROS 2 Package for Swarm Robot Behaviors \- arXiv, accessed on December 11, 2025, [https://arxiv.org/html/2405.02438v1](https://arxiv.org/html/2405.02438v1)  
6. Self-Organizing Aerial Swarm Robotics for Resilient Load Transportation : A Table-Mechanics-Inspired Approach \- arXiv, accessed on December 11, 2025, [https://arxiv.org/html/2509.03563v1](https://arxiv.org/html/2509.03563v1)  
7. CrazyMARL: Decentralized Direct Motor Control Policies for Cooperative Aerial Transport of Cable-Suspended Payloads \- arXiv, accessed on December 11, 2025, [https://arxiv.org/html/2509.14126v1](https://arxiv.org/html/2509.14126v1)  
8. Heavy Lift Drones: A Guide for Drone Pilots, accessed on December 11, 2025, [https://www.droneasaservice.com/blog/heavy-lift-drones-overview/](https://www.droneasaservice.com/blog/heavy-lift-drones-overview/)  
9. Heaviest Drones in the Market: Which UAVs Can Carry the Most? \- Pilot Institute, accessed on December 11, 2025, [https://pilotinstitute.com/heaviest-drones/](https://pilotinstitute.com/heaviest-drones/)  
10. Review of Aerial Transportation of Suspended-Cable Payloads with Quadrotors \- MDPI, accessed on December 11, 2025, [https://www.mdpi.com/2504-446X/8/2/35](https://www.mdpi.com/2504-446X/8/2/35)  
11. Scalable Cooperative Transport of Cable-Suspended Loads with UAVs using Distributed Trajectory Optimization \- Taylor Howell, accessed on December 11, 2025, [https://thowell.github.io/documents/team\_lift\_ral.pdf](https://thowell.github.io/documents/team_lift_ral.pdf)  
12. Drone swarm challenge 2025 | Vinnova, accessed on December 11, 2025, [https://www.vinnova.se/en/calls-for-proposals/radikal-innovation/drone-swarm-challenge-2025/](https://www.vinnova.se/en/calls-for-proposals/radikal-innovation/drone-swarm-challenge-2025/)  
13. Roboticists: Have you used Rust with ROS\[2\]? What were your experiences like? \- Reddit, accessed on December 11, 2025, [https://www.reddit.com/r/rust/comments/173upma/roboticists\_have\_you\_used\_rust\_with\_ros2\_what/](https://www.reddit.com/r/rust/comments/173upma/roboticists_have_you_used_rust_with_ros2_what/)  
14. Rust is for Robotics | robotics.rs, accessed on December 11, 2025, [https://robotics.rs/](https://robotics.rs/)  
15. Current state of Rust client libraries? Which one to use (ros2-client, rus2, ros2\_rust, rclrust, rosrust, or r2r)? \- ROS General \- Open Robotics Discourse, accessed on December 11, 2025, [https://discourse.openrobotics.org/t/current-state-of-rust-client-libraries-which-one-to-use-ros2-client-rus2-ros2-rust-rclrust-rosrust-or-r2r/39119](https://discourse.openrobotics.org/t/current-state-of-rust-client-libraries-which-one-to-use-ros2-client-rus2-ros2-rust-rclrust-rosrust-or-r2r/39119)  
16. Minimizing Discovery Overhead in ROS2 · Zenoh \- pub/sub, geo distributed storage, query, accessed on December 11, 2025, [https://zenoh.io/blog/2021-03-23-discovery/](https://zenoh.io/blog/2021-03-23-discovery/)  
17. Zenoh-Pico: Above and Beyond, accessed on December 11, 2025, [https://zenoh.io/blog/2022-06-09-zenoh-pico-above-and-beyond/](https://zenoh.io/blog/2022-06-09-zenoh-pico-above-and-beyond/)  
18. Overcome DDS Limitations with Zenoh's DDS Plugin and DDS Bridge for Robotic Systems \- YouTube, accessed on December 11, 2025, [https://www.youtube.com/watch?v=9h01\_MSKPS0](https://www.youtube.com/watch?v=9h01_MSKPS0)  
19. Use zenoh-bridge-ros2dds with ROS2 Humble | by William Chen \- Medium, accessed on December 11, 2025, [https://medium.com/@piliwilliam0306/use-zenoh-bridge-ros2dds-with-ros2-humble-459ab70ce9c7](https://medium.com/@piliwilliam0306/use-zenoh-bridge-ros2dds-with-ros2-humble-459ab70ce9c7)  
20. zenoh\_bridge\_dds: Kilted 0.5.0 documentation, accessed on December 11, 2025, [https://docs.ros.org/en/kilted/p/zenoh\_bridge\_dds/](https://docs.ros.org/en/kilted/p/zenoh_bridge_dds/)  
21. Robot Web Tools, accessed on December 11, 2025, [https://robotwebtools.github.io/](https://robotwebtools.github.io/)  
22. ROS2 User Interface : r/ROS \- Reddit, accessed on December 11, 2025, [https://www.reddit.com/r/ROS/comments/1j8t8px/ros2\_user\_interface/](https://www.reddit.com/r/ROS/comments/1j8t8px/ros2_user_interface/)  
23. fzyzcjy/flutter\_rust\_bridge: Flutter/Dart \<-\> Rust binding generator, feature-rich, but seamless and simple. \- GitHub, accessed on December 11, 2025, [https://github.com/fzyzcjy/flutter\_rust\_bridge](https://github.com/fzyzcjy/flutter_rust_bridge)  
24. Using Flutter Rust bridge in 2023 | Roman Zaynetdinov (zaynetro), accessed on December 11, 2025, [https://www.zaynetro.com/post/flutter-rust-bridge-2023](https://www.zaynetro.com/post/flutter-rust-bridge-2023)  
25. Jetson Orin Nano Developer Kit Getting Started Guide, accessed on December 11, 2025, [https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)  
26. Jetson Orin Nano Super Developer Kit \- NVIDIA, accessed on December 11, 2025, [https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)  
27. X500 V2 Kits – Holybro Store, accessed on December 11, 2025, [https://holybro.com/products/x500-v2-kits](https://holybro.com/products/x500-v2-kits)  
28. PX4 Development Kit \- X500 v2 – Holybro Store, accessed on December 11, 2025, [https://holybro.com/products/px4-development-kit-x500-v2](https://holybro.com/products/px4-development-kit-x500-v2)  
29. Holybro Pixhawk 6C Flight Controller \- NewBeeDrone, accessed on December 11, 2025, [https://newbeedrone.com/products/holybro-pixhawk-6c-flight-controller](https://newbeedrone.com/products/holybro-pixhawk-6c-flight-controller)  
30. Pixhawk 6C – Holybro Store, accessed on December 11, 2025, [https://holybro.com/products/pixhawk-6c](https://holybro.com/products/pixhawk-6c)  
31. Luxonis vs. RealSense: Which depth camera should I choose? \- Génération Robots \- Blog, accessed on December 11, 2025, [https://www.generationrobots.com/blog/en/luxonis-vs-realsense-which-depth-camera-should-i-choose/](https://www.generationrobots.com/blog/en/luxonis-vs-realsense-which-depth-camera-should-i-choose/)  
32. OAK-D Pro vs. Intel RealSense D435i \- Blog \- Luxonis Forum, accessed on December 11, 2025, [https://discuss.luxonis.com/blog/1303-oak-d-pro-vs-intel-realsense-d435i](https://discuss.luxonis.com/blog/1303-oak-d-pro-vs-intel-realsense-d435i)  
33. Here 3 GNSS(M8P) GPS Unit \- Foxtech, accessed on December 11, 2025, [https://www.foxtechfpv.com/here-2-gnss-m8n-gps-unit.html](https://www.foxtechfpv.com/here-2-gnss-m8n-gps-unit.html)  
34. Here 3+ RTK GNSS/GPS combo \- Aeroboticshop.com, accessed on December 11, 2025, [https://aeroboticshop.com/products/here-3-rtk-gnss-gps-combo](https://aeroboticshop.com/products/here-3-rtk-gnss-gps-combo)  
35. ROS2swarm/ROS2swarm: A ROS 2 package providing an easy-to-extend framework for and library of swarm behaviors. \- GitHub, accessed on December 11, 2025, [https://github.com/ROS2swarm/ROS2swarm](https://github.com/ROS2swarm/ROS2swarm)  
36. raft\_consensus \- Rust \- Docs.rs, accessed on December 11, 2025, [https://docs.rs/raft-consensus](https://docs.rs/raft-consensus)  
37. raft \- crates.io: Rust Package Registry, accessed on December 11, 2025, [https://crates.io/crates/raft](https://crates.io/crates/raft)  
38. GAIA 160MP-Heavy Lift Drone Frame \- Foxtech, accessed on December 11, 2025, [https://www.foxtechfpv.com/gaia-160-mp-heavy-lift-drone-frame.html](https://www.foxtechfpv.com/gaia-160-mp-heavy-lift-drone-frame.html)  
39. Industrial UAV Frame Kits, Professional Drone Frame Manufacturer \- FOXTECH Store, accessed on December 11, 2025, [https://store.foxtech.com/accessory/frame/](https://store.foxtech.com/accessory/frame/)  
40. T-MOTOR U10 II KV100-Free Shipping \- Foxtech, accessed on December 11, 2025, [https://www.foxtechfpv.com/t-motor-u10-ii-kv100.html](https://www.foxtechfpv.com/t-motor-u10-ii-kv100.html)  
41. Calculating How Much Weight a Drone Can Carry | Unmanned Systems Technology, accessed on December 11, 2025, [https://www.unmannedsystemstechnology.com/feature/calculating-how-much-weight-a-drone-can-carry/](https://www.unmannedsystemstechnology.com/feature/calculating-how-much-weight-a-drone-can-carry/)  
42. Gens Ace- Tattu Plus 1.0, 22000mAh 25C 12S1P 44.4V Lipo with AS150U \- 3DXR, accessed on December 11, 2025, [https://www.3dxr.co.uk/electronics-c78/batteries-c144/gensace-gens-ace-tattu-plus-1-0-22000mah-25c-12s1p-44-4v-lipo-with-as150u-p3447](https://www.3dxr.co.uk/electronics-c78/batteries-c144/gensace-gens-ace-tattu-plus-1-0-22000mah-25c-12s1p-44-4v-lipo-with-as150u-p3447)  
43. Tattu plus UAV & Drone Smart Lipo Battery \- Genstattu.com, accessed on December 11, 2025, [https://genstattu.com/tattu-plus-battery.html](https://genstattu.com/tattu-plus-battery.html)  
44. Skyrc PC3000 3000W 60A 4 Channel Balance Charger for 12s 14s Lipo 4.2V \- LittoHot, accessed on December 11, 2025, [https://www.littohot.com/products/skyrc-pc3000-3000w-60a-4-channel-balance-charger-for-12s-14s-lipo-4-2v-lihv-4-35v-lithium-battery-for-agriculture-drone](https://www.littohot.com/products/skyrc-pc3000-3000w-60a-4-channel-balance-charger-for-12s-14s-lipo-4-2v-lihv-4-35v-lithium-battery-for-agriculture-drone)  
45. Skyrc PC3000 2500W 50A Battery Charger 4 Channel 12/14S Smart High-power Fast Charger \- arrishobby, accessed on December 11, 2025, [https://www.arrishobby.com/products/skyrc-pc3000-2500w-50a-battery-charger-4-channel-12-14s-smart-high-power-fast-charger](https://www.arrishobby.com/products/skyrc-pc3000-2500w-50a-battery-charger-4-channel-12-14s-smart-high-power-fast-charger)  
46. SIYI HM30 Long Range Full HD Digital Image Transmission FPV System \- UnmannedRC, accessed on December 11, 2025, [https://unmannedrc.com/products/siyi-hm30-long-range-full-hd-digital-image-transmission-fpv-system](https://unmannedrc.com/products/siyi-hm30-long-range-full-hd-digital-image-transmission-fpv-system)  
47. Specific Operations Risk Assessment (SORA) \- EASA \- European Union, accessed on December 11, 2025, [https://www.easa.europa.eu/en/domains/drones-air-mobility/operating-drone/specific-category-civil-drones/specific-operations-risk-assessment-sora](https://www.easa.europa.eu/en/domains/drones-air-mobility/operating-drone/specific-category-civil-drones/specific-operations-risk-assessment-sora)  
48. DRONE MARKET IN POLAND \- Instytut Lotnictwa, accessed on December 11, 2025, [https://ilot.lukasiewicz.gov.pl/wp-content/uploads/2024/02/The-Storm-report.pdf](https://ilot.lukasiewicz.gov.pl/wp-content/uploads/2024/02/The-Storm-report.pdf)