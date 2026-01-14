# DAS-SAR: נחיל חיפוש והצלה אווירי מבוזר

🌍 **[English](README.md)** | 🇵🇱 **[Polski](README_PL.md)** | 🇺🇦 **[Українська](README_UA.md)** | 🇮🇱 **[עברית]**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FJazzy-blue.svg)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Language](https://img.shields.io/badge/Language-Python-blue.svg)](https://www.python.org/)
[![Middleware](https://img.shields.io/badge/Middleware-Zenoh-green.svg)](https://zenoh.io/)

## 📖 סקירה כללית

**DAS-SAR** (Distributed Aerial Search and Rescue) מייצג שינוי פרדיגמה מפינוי אווירי מסורתי בעל נקודת כשל בודדת (כמו מסוקים) למערכת **Fail-Operational Distributed Lift System (DLS)** - מערכת הרמה מבוזרת חסינת תקלות. במקום להסתמך על כלי טיס מאסיבי אחד, אנו משתמשים בנחיל של רחפני משא כבדים אוטונומיים הקשורים למטען יחיד.

מערכת זו תוכננה לא רק לחיפוש וסיור, אלא ל**הרמה כבדה יתירה**. על ידי הפרדת כושר ההרמה מגודל כלי הטיס הבודד, נחיל ה-DAS-SAR יכול לחלץ נפגעים (מטענים של 100 ק"ג+) מסביבות בלתי נגישות לכלי טיס קונבנציונליים, כגון יערות עבותים, קניונים עמוקים או אזורי אסון עירוניים.

---

## 📐 ויזואליזציה של המערכת

> **[מקום לדיאגרמה: תרשים המראה נחיל של 6 סוכנים קשורים למטען אלונקה יחיד, הממחיש את הסמכות הגיאומטרית ואת גיאומטריית הכבלים.]**

---

## 🚀 תכונות עיקריות

*   **הרמה מבוזרת חסינת תקלות:** הנחיל שורד אובדן מוחלט של סוכן במהלך משימה מבלי להפיל את המטען.
*   **יתירות של 6 סוכנים:** שלב 2 משתמש במינימום של 6 רחפני משא כבדים (אוקטוקופטרים קואקסיאליים). זה מספק את הסמכות הגיאומטרית הדרושה לשמירה על בקרת 6-DOF (שש דרגות חופש) גם במצב של כשל.
*   **החלפת סוכנים דינמית:** תומך בלוגיקת "Hot-Swap" שבה רחפני מילואים יכולים להחליף סוכנים שהתעייפו או נכשלו במהלך משימה, מה שמבטיח פעילות רציפה 24/7.
*   **בקרת אדמיטנס מבוזרת (Distributed Admittance Control):** מיישם מודלים של מסה-קפיץ-מרסן לייצוב מטענים תלויים וניהול מתח הכבלים ללא לחימה קשיחה על מיקום.
*   **רשת מש (Zenoh):** מינוף **Eclipse Zenoh** לתיאום נחיל בשחיקה נמוכה במיוחד (Ultra-low latency), ביצועים העולים משמעותית על DDS סטנדרטי בסביבות עמוסות או רחבות היקף.

## 🏗 מבנה הפרויקט

```text
.
├── docker/                 # קונפיגורציית Zenoh והגדרות docker
├── docs/                   # תוכניות פרויקט, מחקרים טכניים וניתוח SORA
├── sar_swarm_ws/           # ROS 2 Workspace
│   └── src/
│       ├── heavy_lift_core/# לוגיקת ליבה של הנפת הנחיל (Rust)
│       ├── px4_msgs/       # הגדרות הודעות PX4-ROS 2
│       ├── sar_perception/ # צמתי AI/חזות (זיהוי ומיקום - Python)
│       ├── sar_simulation/ # סימולציית נחיל וסקריפטים לבדיקה
│       └── sar_swarm_control/ # אלגוריתמי בקרה מבוזרים (Rust)
├── Dockerfile              # מכולת סביבת פיתוח
├── docker-compose.yml      # תזמור רב-מכולות
├── README.md               # קובץ זה
├── README_PL.md            # תיעוד בפולנית
├── README_UA.md            # תיעוד באוקראינית
├── README_HE.md            # תיעוד בעברית
├── ROADMAP.md              # לוח זמנים של הפרויקט ואבני דרך
├── TESTING.md              # נהלים ומדריכי בדיקה
└── AGENTS.md               # הקשר טכני ומדריך פיתוח
```

## 🛠 טכנולוגיות

| רכיב | טכנולוגיה |
| :--- | :--- |
| **בקרה קריטית לבטיחות** | **Rust** (rclrs, MAVSDK-Rust) |
| **AI וחזון ממוחשב** | **Python** (PyTorch, YOLOv8/11) |
| **Middleware** | **Eclipse Zenoh** & **ROS 2** (Humble/Jazzy) |
| **סימולציה** | Gazebo Harmonic / PX4 SITL |
| **חומרה (שלב 1)** | NVIDIA Jetson Orin Nano, Pixhawk 6C, Holybro X500 V2 |
| **חומרה (שלב 2)** | T-Motor U15 II / Hobbywing X9 Plus (Coaxial X8) |

## 🚦 תחילת עבודה

### דרישות קדם
- Docker & Docker Compose
- Ubuntu 22.04 LTS (מומלץ)
- ROS 2 Humble/Jazzy
- Rust Toolchain

### הרצת סימולציה
הפרויקט כולל סימולציה מבוססת Docker לבדיקת התנהגות הנחיל מבוססת Rust.

1. **בניית סביבת ה-Docker:**
   ```bash
   docker-compose build
   ```

2. **הפעלת סימולציית הנחיל:**
   ```bash
   docker-compose up
   ```

3. **ויזואליזציה על המארח:**
   ```bash
   python3 visualize_on_host.py
   ```

## 🧪 פיתוח

### הרצת בדיקות יחידה (Unit Tests)
לאימות לוגיקת הבקרה המרכזית:
```bash
cd sar_swarm_ws/src/sar_swarm_control
cargo test
```

## 📖 תיעוד

תיעוד הפרויקט זמין במספר שפות ופורמטים.

### תיעוד ליבה
*   **[Project Plan](docs/Drone%20Swarm%20Evacuation%20Project%20Plan.md)** ([PL](docs/Drone%20Swarm%20Evacuation%20Project%20Plan_PL.md) / [UA](docs/Drone%20Swarm%20Evacuation%20Project%20Plan_UA.md)) — אסטרטגיה ומפרטי שלב 2.
*   **[Technical Architecture](docs/Technical%20Architecture.md)** ([PL](docs/Technical%20Architecture_PL.md) / [UA](docs/Technical%20Architecture_UA.md)) — צלילה עמוקה לתורת הבקרה ותכנון המערכת.
*   **[SORA Safety Case](docs/Safety_Case_SORA.md)** ([PL](docs/Safety_Case_SORA_PL.md) / [UA](docs/Safety_Case_SORA_UA.md)) — ניתוח סיכונים לחילוץ אנושי.
*   **[Roadmap](ROADMAP.md)** — לוח זמנים של הפרויקט ואבני דרך.
*   **[Testing Guide](TESTING.md)** — נהלים לאימות התנהגות הנחיל.
*   **[Agent Development Guide](AGENTS.md)** — הקשר טכני למפתחים.

### מאמרים טכניים (PDF)
*   [Drone Development Without Drones](docs/Drone%20Development%20Without%20Drones.pdf) (EN)
*   [Zaawansowane Środowiska Symulacyjne dla Autonomicznych Rojów Robotów](docs/Zaawansowane%20Środowiska%20Symulacyjne%20dla%20Autonomicznych%20Rojów%20Robotów.pdf) (PL)

## 👥 מחברים ויצירת קשר

- **beret** - [beret@hipisi.org.pl](mailto:beret@hipisi.org.pl)
- **Marysia Software Limited** - [ceo@marysia.app](mailto:ceo@marysia.app)
- **אתר אינטרנט:** [https://marysia.app](https://marysia.app)

---

## ⚖️ משפטי ובטיחות
פעולות המערבות נחילי משא כבד ותחבורה אנושית כפופות לתקנות EASA בקטגוריית Specific/Certified. כל הפעולות חייבות לעקוב אחר פרוטוקולי ה-[SORA](docs/Safety_Case_SORA.md) המוגדרים בתיעוד.
