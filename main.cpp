#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdio>

const float PI = 3.14159265358979323846f;

struct Station {
    sf::Vector2f pos;
    int id;
    float radius;
};

struct Box {
    sf::Vector2f pos;
    int id;
    float size;
    bool isPicked;
};

struct Obstacle {
    sf::Vector2f pos;
    float radius;
};

struct SubTask {
    int stationId;
    char action; // 'k': Al (Pick), 'b': Birak (Drop)
};

// 1 Saniyede Bir Gerçek Zamanlı Örnekleyen Dinamik BMS / ADC Sensörü
struct BatterySensor {
    float capacity_mAh = 10000.0f;
    float remaining_mAh = 10000.0f;
    float currentDraw_A = 0.35f;
    float sampledVoltage = 25.20f;
    float sampledPercentage = 100.0f;
    float sampleTimer = 0.0f;

    void Update(float dt, bool isDriving, float loadKg) {
        if (isDriving) {
            currentDraw_A = 1.70f + (loadKg * 0.25f);
        } else {
            currentDraw_A = 0.35f;
        }

        remaining_mAh -= (currentDraw_A * 1000.0f) * (dt / 3600.0f);
        if (remaining_mAh < 0.0f) remaining_mAh = 0.0f;

        sampleTimer += dt;
        if (sampleTimer >= 1.0f) {
            sampleTimer = 0.0f;
            float soc = (remaining_mAh / capacity_mAh);
            float noise = ((float)(rand() % 100 - 50) / 1000.0f) * 0.03f;
            sampledVoltage = (21.0f + soc * 4.2f) - (currentDraw_A * 0.04f) + noise;
            sampledPercentage = std::clamp(soc * 100.0f, 0.0f, 100.0f);
        }
    }
};

struct Robot {
    sf::Vector2f pos;
    sf::Vector2f homePos;
    float angle;
    float speed;
    float baseSpeed;
    float width;
    float length;
    bool isRunning;
    bool hasMission;
    bool isReturningHome;

    // Kamera ve Engel Durumları
    bool frontObstacle, leftObstacle, rightObstacle;
    bool frontWarning;
    bool leftSeesLine, rightSeesLine, frontSeesLine;
    float leftLineAngleOffset, rightLineAngleOffset, frontLineAngleOffset;
    bool frontSeesTarget, leftSeesTarget, rightSeesTarget;
    bool obstacleDetected;

    // Manevra Durumları
    bool isDoingUTurn;
    float uTurnTimer;
    float branchCooldown;
    int lastTurnDir;

    float payloadKg;
    std::vector<int> carriedBoxIds;
    float visionConfidence;

    // 5'li Çizgi Sensörü
    bool sensorStates[5];
    sf::Vector2f sensorPositions[5];

    // Görev ve Bekleme
    std::vector<SubTask> taskQueue;
    size_t currentTaskIdx;
    float taskWaitTimer;
    bool isWaitingForBox;
};

float NormalizeAngle(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

void MarkGridCircle(std::vector<uint8_t>& grid, int w, int h, int cx, int cy, int r) {
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx*dx + dy*dy <= r*r) {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < w && y >= 0 && y < h) {
                    grid[y * w + x] = 1;
                }
            }
        }
    }
}

void DrawTrackLine(sf::RenderTexture& canvas, std::vector<uint8_t>& grid, int w, int h, sf::Vector2f p1, sf::Vector2f p2, float brushRadius) {
    float dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int steps = std::max(1, (int)(dist / 2.0f));
    for (int i = 0; i <= steps; ++i) {
        float t = (float)i / steps;
        float ix = p1.x + t * (p2.x - p1.x);
        float iy = p1.y + t * (p2.y - p1.y);

        sf::CircleShape dot(brushRadius);
        dot.setOrigin(brushRadius, brushRadius);
        dot.setPosition(ix, iy);
        dot.setFillColor(sf::Color(20, 20, 20));
        canvas.draw(dot);
        MarkGridCircle(grid, w, h, (int)ix, (int)iy, (int)brushRadius);
    }
    canvas.display();
}

sf::Vector2f SnapToNearestTrack(sf::Vector2f pt, const std::vector<uint8_t>& grid, int w, int h, int maxSearchRadius = 90) {
    int cx = std::clamp((int)pt.x, 0, w - 1);
    int cy = std::clamp((int)pt.y, 0, h - 1);

    float sumX = 0.0f, sumY = 0.0f;
    int count = 0;

    int centroidRadius = 14;
    for (int dy = -centroidRadius; dy <= centroidRadius; dy += 2) {
        for (int dx = -centroidRadius; dx <= centroidRadius; dx += 2) {
            int x = cx + dx;
            int y = cy + dy;
            if (x >= 0 && x < w && y >= 0 && y < h && grid[y * w + x] == 1) {
                sumX += (float)x;
                sumY += (float)y;
                count++;
            }
        }
    }
    if (count > 0) {
        return sf::Vector2f(sumX / count, sumY / count);
    }

    float bestDistSq = 1e9f;
    sf::Vector2f bestPt = pt;
    bool found = false;

    for (int r = 4; r <= maxSearchRadius; r += 3) {
        for (int dy = -r; dy <= r; dy += 3) {
            for (int dx = -r; dx <= r; dx += 3) {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < w && y >= 0 && y < h && grid[y * w + x] == 1) {
                    float d2 = (float)(dx * dx + dy * dy);
                    if (d2 < bestDistSq) {
                        bestDistSq = d2;
                        bestPt = sf::Vector2f((float)x, (float)y);
                        found = true;
                    }
                }
            }
        }
        if (found) break;
    }
    return bestPt;
}

bool IsObstacleOnTrack(const Obstacle& obs, const std::vector<uint8_t>& grid, int w, int h) {
    int r = (int)obs.radius + 2;
    int cx = (int)obs.pos.x;
    int cy = (int)obs.pos.y;
    for (int dy = -r; dy <= r; dy += 3) {
        for (int dx = -r; dx <= r; dx += 3) {
            if (dx * dx + dy * dy <= r * r) {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < w && y >= 0 && y < h && grid[y * w + x] == 1) {
                    return true;
                }
            }
        }
    }
    return false;
}

void EvaluateFrontObstacles(
    sf::Vector2f camOrigin, float robotAngle, float robotWidth, float fovRad, float fullRange,
    const std::vector<Obstacle>& obstacles, const std::vector<uint8_t>& grid, int w, int h,
    bool& outStopHalfRange, bool& outWarningFullRange)
{
    outStopHalfRange = false;
    outWarningFullRange = false;

    float halfRange = fullRange * 0.5f;
    sf::Vector2f forward(cosf(robotAngle), sinf(robotAngle));
    sf::Vector2f right(-sinf(robotAngle), cosf(robotAngle));

    for (const auto& obs : obstacles) {
        sf::Vector2f toObs = obs.pos - camOrigin;
        float centerDist = std::hypot(toObs.x, toObs.y);
        float surfaceDist = std::max(0.0f, centerDist - obs.radius);

        if (surfaceDist > fullRange) continue;

        float angleToObs = atan2f(toObs.y, toObs.x);
        float diff = NormalizeAngle(angleToObs - robotAngle);
        if (fabsf(diff) > (fovRad / 2.0f)) continue;

        bool onTrack = IsObstacleOnTrack(obs, grid, w, h);
        float forwardProj = toObs.x * forward.x + toObs.y * forward.y;
        float lateralDist = fabsf(toObs.x * right.x + toObs.y * right.y);
        bool inPhysicalCorridor = (forwardProj > -5.0f) && (lateralDist < (robotWidth * 0.5f + obs.radius + 3.0f));

        if (!onTrack && !inPhysicalCorridor) continue;
        if (lateralDist > (robotWidth * 0.85f + obs.radius)) continue;

        outWarningFullRange = true;
        if (surfaceDist <= halfRange) {
            outStopHalfRange = true;
            break;
        }
    }
}

bool CheckSideRelevantObstacle(sf::Vector2f origin, float baseAngle, float fovRad, float range, const std::vector<Obstacle>& obstacles, const std::vector<uint8_t>& grid, int w, int h) {
    float halfRange = range * 0.5f;
    for (const auto& obs : obstacles) {
        if (!IsObstacleOnTrack(obs, grid, w, h)) continue;
        float dist = std::hypot(obs.pos.x - origin.x, obs.pos.y - origin.y) - obs.radius;
        if (dist <= halfRange) {
            float angleToObs = atan2f(obs.pos.y - origin.y, obs.pos.x - origin.x);
            float diff = NormalizeAngle(angleToObs - baseAngle);
            if (fabsf(diff) <= (fovRad / 2.0f)) return true;
        }
    }
    return false;
}

bool CheckConeStation(sf::Vector2f origin, float baseAngle, float fovRad, float range, sf::Vector2f stPos, float stRadius) {
    float dist = std::hypot(stPos.x - origin.x, stPos.y - origin.y);
    if (dist < range + stRadius) {
        float angleToSt = atan2f(stPos.y - origin.y, stPos.x - origin.x);
        float diff = NormalizeAngle(angleToSt - baseAngle);
        if (fabsf(diff) <= (fovRad / 2.0f) + 0.12f) {
            return true;
        }
    }
    return false;
}

bool ScanConeForLine(sf::Vector2f origin, float robotAngle, float camAngle, float fovRad, float minRange, float maxRange,
                     const std::vector<uint8_t>& grid, int w, int h, float& outAngleOffset)
{
    float sumSin = 0.0f, sumCos = 0.0f;
    int hitCount = 0;

    const int angleSteps = 9;
    const int distSteps = 6;

    for (int ia = 0; ia <= angleSteps; ++ia) {
        float a = (camAngle - fovRad * 0.45f) + (fovRad * 0.9f) * ((float)ia / angleSteps);
        for (int id = 0; id < distSteps; ++id) {
            float d = minRange + (maxRange - minRange) * ((float)id / (distSteps - 1));
            int sx = (int)(origin.x + cosf(a) * d);
            int sy = (int)(origin.y + sinf(a) * d);
            if (sx >= 0 && sx < w && sy >= 0 && sy < h && grid[sy * w + sx] == 1) {
                float relAngle = NormalizeAngle(a - robotAngle);
                sumCos += cosf(relAngle);
                sumSin += sinf(relAngle);
                hitCount++;
            }
        }
    }

    if (hitCount >= 2) {
        outAngleOffset = atan2f(sumSin, sumCos);
        return true;
    }
    outAngleOffset = 0.0f;
    return false;
}

void DrawCameraCone(sf::RenderWindow& win, sf::Vector2f origin, float baseAngle, float fovRad, float range, bool hasObstacle, bool hasWarning, bool seesTarget, sf::Color normalCol, bool drawHalfLine = false) {
    sf::ConvexShape cone;
    const int segments = 20;
    cone.setPointCount(segments + 2);
    cone.setPoint(0, origin);

    float aStart = baseAngle - fovRad / 2.0f;
    float aEnd = baseAngle + fovRad / 2.0f;

    for (int i = 0; i <= segments; ++i) {
        float ang = aStart + (float)i / segments * (aEnd - aStart);
        cone.setPoint(i + 1, origin + sf::Vector2f(cosf(ang), sinf(ang)) * range);
    }

    if (hasObstacle) {
        cone.setFillColor(sf::Color(255, 40, 40, 125));
    } else if (hasWarning) {
        cone.setFillColor(sf::Color(255, 165, 0, 85));
    } else if (seesTarget) {
        cone.setFillColor(sf::Color(255, 215, 0, 95));
    } else {
        cone.setFillColor(normalCol);
    }
    win.draw(cone);

    if (drawHalfLine) {
        float halfR = range * 0.5f;
        sf::VertexArray arc(sf::LineStrip, segments + 1);
        for (int i = 0; i <= segments; ++i) {
            float ang = aStart + (float)i / segments * (aEnd - aStart);
            sf::Vector2f pt = origin + sf::Vector2f(cosf(ang), sinf(ang)) * halfR;
            arc[i] = sf::Vertex(pt, hasObstacle ? sf::Color(255, 0, 0, 220) : sf::Color(255, 255, 255, 130));
        }
        win.draw(arc);
    }
}

bool LoadReliableFont(sf::Font& font) {
    std::vector<std::string> candidates = {
        "./font.ttf",
        "/usr/share/fonts/liberation-mono-fonts/LiberationMono-Regular.ttf",
        "/usr/share/fonts/liberation-sans-fonts/LiberationSans-Regular.ttf",
        "/usr/share/fonts/liberation-mono/LiberationMono-Regular.ttf",
        "/usr/share/fonts/liberation-sans/LiberationSans-Regular.ttf",
        "/usr/share/fonts/gnu-free/FreeSans.ttf",
        "/usr/share/fonts/gnu-free/FreeMono.ttf"
    };

    FILE* pipe = popen("fc-list : file | grep -iE '\\.ttf$' | head -n 30", "r");
    if (pipe) {
        char buf[512];
        while (fgets(buf, sizeof(buf), pipe)) {
            std::string p = buf;
            while (!p.empty() && (p.back() == '\n' || p.back() == '\r' || p.back() == ':')) p.pop_back();
            if (!p.empty()) candidates.push_back(p);
        }
        pclose(pipe);
    }

    for (const auto& path : candidates) {
        if (font.loadFromFile(path)) return true;
    }
    return false;
}

int main() {
    const int simW = 960, simH = 750;
    const int dashW = 460, dashH = 750;
    const int termW = 660, termH = 380;

    sf::RenderWindow simWin(sf::VideoMode(simW, simH), "1. AGV Simulasyon Parkuru", sf::Style::Titlebar | sf::Style::Close);
    sf::RenderWindow dataWin(sf::VideoMode(dashW, dashH), "2. AGV Telemetri ve Sensor Verileri", sf::Style::Titlebar | sf::Style::Close);
    sf::RenderWindow termWin(sf::VideoMode(termW, termH), "3. AGV WMS Terminali", sf::Style::Titlebar | sf::Style::Close);

    simWin.setFramerateLimit(60);
    dataWin.setFramerateLimit(60);
    termWin.setFramerateLimit(60);

    simWin.setPosition(sf::Vector2i(30, 40));
    dataWin.setPosition(sf::Vector2i(simW + 50, 40));
    termWin.setPosition(sf::Vector2i(30, simH + 70));

    sf::Font font;
    bool fontOk = LoadReliableFont(font);

    sf::RenderTexture canvas;
    canvas.create(simW, simH);
    canvas.clear(sf::Color(245, 245, 245));
    canvas.display();

    std::vector<uint8_t> trackGrid(simW * simH, 0);
    const float brushRadius = 11.0f;

    sf::Vector2f strokeStartPos(0, 0);
    sf::Vector2f prevFreehandPos(0, 0);
    bool isMouseDown = false;

    std::vector<Station> stations;
    std::vector<Box> boxes;
    std::vector<Obstacle> obstacles;
    int nextStationId = 1, nextBoxId = 1;

    Robot robot;
    robot.pos = sf::Vector2f(200, 200);
    robot.homePos = sf::Vector2f(200, 200);
    robot.angle = 0.0f;
    robot.baseSpeed = 80.0f;
    robot.speed = 0.0f;
    robot.width = 34.0f;
    robot.length = 50.0f;
    robot.isRunning = false;
    robot.hasMission = false;
    robot.isReturningHome = false;
    robot.frontObstacle = robot.leftObstacle = robot.rightObstacle = false;
    robot.frontWarning = false;
    robot.leftSeesLine = robot.rightSeesLine = robot.frontSeesLine = false;
    robot.leftLineAngleOffset = robot.rightLineAngleOffset = robot.frontLineAngleOffset = 0.0f;
    robot.frontSeesTarget = robot.leftSeesTarget = robot.rightSeesTarget = false;
    robot.obstacleDetected = false;
    robot.isDoingUTurn = false;
    robot.uTurnTimer = 0.0f;
    robot.branchCooldown = 0.0f;
    robot.lastTurnDir = 1;
    robot.payloadKg = 0.0f;
    robot.visionConfidence = 98.4f;
    robot.currentTaskIdx = 0;
    robot.taskWaitTimer = 0.0f;
    robot.isWaitingForBox = false;

    BatterySensor battery;

    std::string termInput = "";
    std::vector<std::string> termLogs = {
        "[SISTEM] Aktif Yan Kamera Kilitli AGV Navigasyonu Devrede.",
        "[KAMERA] Yan kameralar cizgi acisini hesaplar, yoldan cikmayi onler.",
        "[ENGEL] Cizgi disi engeller yoksayilir; kamera YARISINDA (%50) durur."
    };
    float cursorBlinkTimer = 0.0f;
    bool showCursor = true;

    sf::Clock deltaClock;

    while (simWin.isOpen() && dataWin.isOpen() && termWin.isOpen()) {
        float dt = deltaClock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;

        if (robot.branchCooldown > 0.0f) robot.branchCooldown -= dt;

        sf::Event ev;
        while (simWin.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed) {
                simWin.close(); dataWin.close(); termWin.close();
            }

            if (ev.type == sf::Event::LostFocus) isMouseDown = false;

            if (ev.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f clickPos = simWin.mapPixelToCoords(sf::Vector2i(ev.mouseButton.x, ev.mouseButton.y));

                if (ev.mouseButton.button == sf::Mouse::Left) {
                    isMouseDown = true;
                    strokeStartPos = clickPos;
                    prevFreehandPos = clickPos;

                    bool shiftPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
                    if (!shiftPressed && clickPos.x >= 0 && clickPos.x < simW && clickPos.y >= 0 && clickPos.y < simH) {
                        sf::CircleShape dot(brushRadius);
                        dot.setOrigin(brushRadius, brushRadius);
                        dot.setPosition(clickPos);
                        dot.setFillColor(sf::Color(20, 20, 20));
                        canvas.draw(dot);
                        canvas.display();
                        MarkGridCircle(trackGrid, simW, simH, (int)clickPos.x, (int)clickPos.y, (int)brushRadius);
                    }
                } else if (ev.mouseButton.button == sf::Mouse::Right) {
                    bool hit = false;
                    for (size_t i = 0; i < obstacles.size(); ++i) {
                        float dist = std::hypot(clickPos.x - obstacles[i].pos.x, clickPos.y - obstacles[i].pos.y);
                        if (dist < obstacles[i].radius) {
                            obstacles.erase(obstacles.begin() + i);
                            hit = true;
                            termLogs.push_back("[RADAR] Engel kaldirildi.");
                            break;
                        }
                    }
                    if (!hit) {
                        obstacles.push_back({ clickPos, 22.0f });
                        bool onLine = IsObstacleOnTrack(obstacles.back(), trackGrid, simW, simH);
                        termLogs.push_back(onLine ? "[RADAR] Cizgi uzerine engel eklendi (Aktif Tehdit)." :
                                                    "[RADAR] Cizgi disina engel eklendi (Robot yoksayacak).");
                    }
                }
            }

            if (ev.type == sf::Event::MouseButtonReleased) {
                if (ev.mouseButton.button == sf::Mouse::Left && isMouseDown) {
                    sf::Vector2f releasePos = simWin.mapPixelToCoords(sf::Vector2i(ev.mouseButton.x, ev.mouseButton.y));
                    bool shiftPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);

                    if (shiftPressed) {
                        float dx = releasePos.x - strokeStartPos.x;
                        float dy = releasePos.y - strokeStartPos.y;
                        sf::Vector2f snappedEnd = (fabsf(dx) >= fabsf(dy)) ? sf::Vector2f(releasePos.x, strokeStartPos.y)
                                                                           : sf::Vector2f(strokeStartPos.x, releasePos.y);

                        DrawTrackLine(canvas, trackGrid, simW, simH, strokeStartPos, snappedEnd, brushRadius);
                        termLogs.push_back("[CETVEL] Duz cizgi basariyla cizildi.");
                    }
                    isMouseDown = false;
                }
            }

            if (ev.type == sf::Event::KeyPressed) {
                sf::Vector2f keyMouse = simWin.mapPixelToCoords(sf::Mouse::getPosition(simWin));

                if (ev.key.code == sf::Keyboard::T) {
                    sf::Vector2f snappedSt = SnapToNearestTrack(keyMouse, trackGrid, simW, simH, 45);
                    stations.push_back({ snappedSt, nextStationId++, 30.0f });
                }
                if (ev.key.code == sf::Keyboard::K) {
                    boxes.push_back({ keyMouse, nextBoxId++, 24.0f, false });
                    termLogs.push_back("[DEPO] K" + std::to_string(nextBoxId - 1) + " kutusu yerlestirildi.");
                }

                if (ev.key.code == sf::Keyboard::R) {
                    sf::Vector2f snappedHome = SnapToNearestTrack(keyMouse, trackGrid, simW, simH, 100);
                    robot.pos = snappedHome;
                    robot.homePos = snappedHome;
                    robot.angle = 0.0f;
                    robot.isRunning = false;
                    robot.hasMission = false;
                    robot.isReturningHome = false;
                    robot.taskQueue.clear();
                    robot.currentTaskIdx = 0;
                    robot.isWaitingForBox = false;
                    robot.isDoingUTurn = false;
                    termLogs.push_back("[KONUM] Robot ve HOME cizgiye kilitlendi (Aci: 0.0 deg).");
                }
                if (ev.key.code == sf::Keyboard::C) {
                    canvas.clear(sf::Color(245, 245, 245));
                    canvas.display();
                    std::fill(trackGrid.begin(), trackGrid.end(), 0);
                    stations.clear(); boxes.clear(); obstacles.clear();
                    nextStationId = 1; nextBoxId = 1;
                    robot.isRunning = false; robot.hasMission = false;
                    robot.isReturningHome = false;
                    robot.taskQueue.clear(); robot.payloadKg = 0.0f;
                    robot.carriedBoxIds.clear();
                    robot.isWaitingForBox = false;
                    robot.isDoingUTurn = false;
                    termLogs.push_back("[TEMIZLIK] Parkur ve nesneler sifirlandi.");
                }
            }
        }

        bool shiftHeld = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
        if (isMouseDown && !shiftHeld && simWin.hasFocus()) {
            sf::Vector2f curMouse = simWin.mapPixelToCoords(sf::Mouse::getPosition(simWin));
            if (curMouse.x >= 0 && curMouse.x < simW && curMouse.y >= 0 && curMouse.y < simH) {
                float dist = std::hypot(curMouse.x - prevFreehandPos.x, curMouse.y - prevFreehandPos.y);
                if (dist > 2.0f) {
                    DrawTrackLine(canvas, trackGrid, simW, simH, prevFreehandPos, curMouse, brushRadius);
                    prevFreehandPos = curMouse;
                }
            }
        }

        while (dataWin.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed) {
                simWin.close(); dataWin.close(); termWin.close();
            }
        }

        while (termWin.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed) {
                simWin.close(); dataWin.close(); termWin.close();
            }
            if (ev.type == sf::Event::TextEntered) {
                if (ev.text.unicode == 8) {
                    if (!termInput.empty()) termInput.pop_back();
                } else if (ev.text.unicode == 13 || ev.text.unicode == 10) {
                    if (!termInput.empty()) {
                        std::string cmd = termInput;
                        termLogs.push_back("AGV-WMS> " + cmd);

                        std::string cleanCmd = "";
                        for (char c : cmd) {
                            if (c == '/' || c == ',' || c == ';' || c == '|') cleanCmd += ' ';
                            else cleanCmd += std::tolower(c);
                        }

                        std::stringstream ss(cleanCmd);
                        std::string token;
                        std::vector<SubTask> parsed;

                        while (ss >> token) {
                            if (token.length() >= 3 && token[0] == 't') {
                                char action = token.back();
                                if (action == 'k' || action == 'b') {
                                    std::string numStr = token.substr(1, token.length() - 2);
                                    bool allDigits = !numStr.empty() && std::all_of(numStr.begin(), numStr.end(), ::isdigit);
                                    if (allDigits) {
                                        try {
                                            int stId = std::stoi(numStr);
                                            parsed.push_back({ stId, action });
                                        } catch (...) {}
                                    }
                                }
                            }
                        }

                        if (!parsed.empty()) {
                            robot.homePos = SnapToNearestTrack(robot.homePos, trackGrid, simW, simH, 100);
                            robot.pos = SnapToNearestTrack(robot.pos, trackGrid, simW, simH, 100);
                            robot.taskQueue = parsed;
                            robot.currentTaskIdx = 0;
                            robot.hasMission = true;
                            robot.isRunning = true;
                            robot.isReturningHome = false;
                            robot.taskWaitTimer = 0.0f;
                            robot.isWaitingForBox = false;
                            robot.isDoingUTurn = false;

                            std::string planSummary = "[PLAN] ";
                            for (size_t i = 0; i < parsed.size(); ++i) {
                                planSummary += "T" + std::to_string(parsed[i].stationId) + (parsed[i].action == 'k' ? "(AL)" : "(BIRAK)");
                                if (i + 1 < parsed.size()) planSummary += " -> ";
                            }
                            planSummary += " -> HOME(PARK)";
                            termLogs.push_back("[WMS EMRI] " + std::to_string(parsed.size()) + " adimlik senaryo yuklendi!");
                            termLogs.push_back(planSummary);
                        } else {
                            termLogs.push_back("[HATA] Gecersiz komut! Ornek: t1k t3b / t2k t4b");
                        }
                        termInput.clear();
                    }
                } else if (ev.text.unicode >= 32 && ev.text.unicode < 127) {
                    termInput += static_cast<char>(ev.text.unicode);
                }
            }
        }

        cursorBlinkTimer += dt;
        if (cursorBlinkTimer >= 0.45f) {
            showCursor = !showCursor;
            cursorBlinkTimer = 0.0f;
        }

        sf::Vector2f forward(cosf(robot.angle), sinf(robot.angle));
        sf::Vector2f right(-sinf(robot.angle), cosf(robot.angle));

        float sensorForwardDist = robot.length / 2.0f + 8.0f;
        float sensorSpacing = 6.5f;
        float offsets[5] = { -2.0f, -1.0f, 0.0f, 1.0f, 2.0f };

        int activeCount = 0;
        float weightedSum = 0.0f;

        for (int i = 0; i < 5; ++i) {
            robot.sensorPositions[i] = robot.pos + forward * sensorForwardDist + right * (offsets[i] * sensorSpacing);
            int sx = (int)robot.sensorPositions[i].x;
            int sy = (int)robot.sensorPositions[i].y;

            if (sx >= 0 && sx < simW && sy >= 0 && sy < simH) {
                robot.sensorStates[i] = (trackGrid[sy * simW + sx] == 1);
            } else {
                robot.sensorStates[i] = false;
            }

            if (robot.sensorStates[i]) {
                activeCount++;
                weightedSum += offsets[i];
            }
        }

        sf::Vector2f frontCamOrigin = robot.pos + forward * (robot.length / 2.0f);
        float frontFov = 1.92f;
        float frontRange = 115.0f;

        sf::Vector2f leftCamOrigin = robot.pos - right * (robot.width / 2.0f);
        float leftAngle = robot.angle - (PI / 2.0f);
        float sideFov = 1.35f;
        float sideRange = 80.0f;

        sf::Vector2f rightCamOrigin = robot.pos + right * (robot.width / 2.0f);
        float rightAngle = robot.angle + (PI / 2.0f);

        EvaluateFrontObstacles(
            frontCamOrigin, robot.angle, robot.width, frontFov, frontRange,
            obstacles, trackGrid, simW, simH,
            robot.frontObstacle, robot.frontWarning
        );
        robot.leftObstacle = CheckSideRelevantObstacle(leftCamOrigin, leftAngle, sideFov, sideRange, obstacles, trackGrid, simW, simH);
        robot.rightObstacle = CheckSideRelevantObstacle(rightCamOrigin, rightAngle, sideFov, sideRange, obstacles, trackGrid, simW, simH);
        robot.obstacleDetected = robot.frontObstacle;

        robot.frontSeesLine = ScanConeForLine(frontCamOrigin, robot.angle, robot.angle, frontFov, 10.0f, 65.0f, trackGrid, simW, simH, robot.frontLineAngleOffset);
        robot.leftSeesLine  = ScanConeForLine(leftCamOrigin,  robot.angle, leftAngle,   sideFov,  8.0f,  68.0f, trackGrid, simW, simH, robot.leftLineAngleOffset);
        robot.rightSeesLine = ScanConeForLine(rightCamOrigin, robot.angle, rightAngle,  sideFov,  8.0f,  68.0f, trackGrid, simW, simH, robot.rightLineAngleOffset);

        if (activeCount > 0 || robot.frontSeesLine) {
            float baseConf = (robot.sensorStates[2]) ? 98.8f : 95.2f;
            float jitter = ((float)(rand() % 30 - 15) / 10.0f);
            robot.visionConfidence = std::clamp(baseConf + jitter, 90.0f, 99.8f);
        } else if (robot.leftSeesLine || robot.rightSeesLine) {
            robot.visionConfidence = 86.5f;
        } else {
            robot.visionConfidence = std::clamp(robot.visionConfidence - dt * 20.0f, 40.0f, 99.0f);
        }

        robot.frontSeesTarget = robot.leftSeesTarget = robot.rightSeesTarget = false;
        sf::Vector2f activeTargetPos = robot.homePos;
        float activeTargetRadius = 26.0f;
        bool hasActiveTarget = false;
        float angleDiffToTarget = 0.0f;
        float distToActiveTarget = 1e9f;

        const Station* currentTargetSt = nullptr;

        if (robot.hasMission && !robot.isReturningHome && robot.currentTaskIdx < robot.taskQueue.size()) {
            int targetId = robot.taskQueue[robot.currentTaskIdx].stationId;
            for (const auto& st : stations) {
                if (st.id == targetId) { currentTargetSt = &st; break; }
            }
            if (currentTargetSt != nullptr) {
                activeTargetPos = currentTargetSt->pos;
                activeTargetRadius = currentTargetSt->radius;
                hasActiveTarget = true;
            }
        } else if (robot.isReturningHome) {
            activeTargetPos = robot.homePos;
            activeTargetRadius = 28.0f;
            hasActiveTarget = true;
        }

        if (hasActiveTarget) {
            distToActiveTarget = std::hypot(activeTargetPos.x - robot.pos.x, activeTargetPos.y - robot.pos.y);
            float targetAngle = atan2f(activeTargetPos.y - robot.pos.y, activeTargetPos.x - robot.pos.x);
            angleDiffToTarget = NormalizeAngle(targetAngle - robot.angle);

            robot.frontSeesTarget = CheckConeStation(frontCamOrigin, robot.angle, frontFov, frontRange, activeTargetPos, activeTargetRadius);
            robot.leftSeesTarget  = CheckConeStation(leftCamOrigin,  leftAngle,   sideFov,  sideRange,  activeTargetPos, activeTargetRadius);
            robot.rightSeesTarget = CheckConeStation(rightCamOrigin, rightAngle,  sideFov,  sideRange,  activeTargetPos, activeTargetRadius);
        }

        bool isExecuting = (robot.hasMission && robot.currentTaskIdx < robot.taskQueue.size() && !robot.isReturningHome);

        if (isExecuting && robot.isRunning && !robot.obstacleDetected && !robot.isDoingUTurn) {
            SubTask cur = robot.taskQueue[robot.currentTaskIdx];

            if (currentTargetSt != nullptr) {
                float d = std::hypot(robot.pos.x - currentTargetSt->pos.x, robot.pos.y - currentTargetSt->pos.y);

                if (d < 24.0f) {
                    robot.speed = 0.0f;

                    if (cur.action == 'k') {
                        Box* availableBox = nullptr;
                        for (auto& bx : boxes) {
                            if (!bx.isPicked && std::hypot(bx.pos.x - currentTargetSt->pos.x, bx.pos.y - currentTargetSt->pos.y) < 55.0f) {
                                availableBox = &bx;
                                break;
                            }
                        }

                        if (availableBox == nullptr) {
                            robot.isWaitingForBox = true;
                            robot.taskWaitTimer = 0.0f;

                            static float waitMsgTimer = 0.0f;
                            waitMsgTimer += dt;
                            if (waitMsgTimer > 3.0f) {
                                waitMsgTimer = 0.0f;
                                termLogs.push_back("[BEKLEME] T" + std::to_string(currentTargetSt->id) + " kutu bekleniyor... ('K' ile kutu koyun)");
                            }
                        } else {
                            robot.isWaitingForBox = false;
                            robot.taskWaitTimer += dt;

                            if (robot.taskWaitTimer >= 1.0f) {
                                availableBox->isPicked = true;
                                robot.carriedBoxIds.push_back(availableBox->id);
                                robot.payloadKg += 5.0f;
                                robot.taskWaitTimer = 0.0f;
                                robot.currentTaskIdx++;
                                robot.branchCooldown = 0.85f;
                                termLogs.push_back("[AGV] T" + std::to_string(currentTargetSt->id) + " kutu alindi! (Yuk: " + std::to_string((int)robot.payloadKg) + " kg).");

                                bool lineAhead = false;
                                for (float fDist = 18.0f; fDist <= 45.0f; fDist += 9.0f) {
                                    int fx = (int)(robot.pos.x + forward.x * (robot.length / 2.0f + fDist));
                                    int fy = (int)(robot.pos.y + forward.y * (robot.length / 2.0f + fDist));
                                    if (fx >= 0 && fx < simW && fy >= 0 && fy < simH && trackGrid[fy * simW + fx] == 1) {
                                        lineAhead = true;
                                        break;
                                    }
                                }
                                if (!lineAhead) {
                                    robot.isDoingUTurn = true;
                                    robot.uTurnTimer = 0.0f;
                                    termLogs.push_back("[MANEVRA] Cep hatti sonu: 180 deg U-donusu yapiliyor...");
                                }
                            }
                        }
                    } else if (cur.action == 'b') {
                        robot.isWaitingForBox = false;
                        robot.taskWaitTimer += dt;

                        if (robot.taskWaitTimer >= 1.0f) {
                            if (!robot.carriedBoxIds.empty()) {
                                int bId = robot.carriedBoxIds.back();
                                robot.carriedBoxIds.pop_back();
                                for (auto& bx : boxes) {
                                    if (bx.id == bId) {
                                        bx.isPicked = false;
                                        bx.pos = currentTargetSt->pos;
                                        break;
                                    }
                                }
                                robot.payloadKg = std::max(0.0f, robot.payloadKg - 5.0f);
                                termLogs.push_back("[AGV] T" + std::to_string(currentTargetSt->id) + " kutu teslim edildi.");
                            }
                            robot.taskWaitTimer = 0.0f;
                            robot.currentTaskIdx++;
                            robot.branchCooldown = 0.85f;

                            bool lineAhead = false;
                            for (float fDist = 18.0f; fDist <= 45.0f; fDist += 9.0f) {
                                int fx = (int)(robot.pos.x + forward.x * (robot.length / 2.0f + fDist));
                                int fy = (int)(robot.pos.y + forward.y * (robot.length / 2.0f + fDist));
                                if (fx >= 0 && fx < simW && fy >= 0 && fy < simH && trackGrid[fy * simW + fx] == 1) {
                                    lineAhead = true;
                                    break;
                                }
                            }
                            if (!lineAhead) {
                                robot.isDoingUTurn = true;
                                robot.uTurnTimer = 0.0f;
                                termLogs.push_back("[MANEVRA] Cep hatti sonu: 180 deg U-donusu yapiliyor...");
                            }
                        }
                    }
                }
            }
        }

        if (robot.hasMission && robot.currentTaskIdx >= robot.taskQueue.size() && !robot.isDoingUTurn && !robot.isReturningHome) {
            robot.isReturningHome = true;
            termLogs.push_back("[WMS] Gorevler bitti! Baslangic noktasina (HOME) donuluyor...");
        }

        if (robot.isReturningHome && !robot.isDoingUTurn) {
            float distToHome = std::hypot(robot.pos.x - robot.homePos.x, robot.pos.y - robot.homePos.y);
            if (distToHome < 28.0f) {
                robot.pos = robot.homePos;
                robot.speed = 0.0f;
                robot.isRunning = false;
                robot.hasMission = false;
                robot.isReturningHome = false;
                robot.angle = 0.0f;
                termLogs.push_back("[BASARI] AGV baslangic noktasina (HOME) ulasti ve park edildi!");
            }
        }

        if (robot.isDoingUTurn) {
            robot.speed = 0.0f;
            robot.pos = SnapToNearestTrack(robot.pos, trackGrid, simW, simH, 40);
            robot.angle = NormalizeAngle(robot.angle + 4.2f * dt);

            robot.uTurnTimer += dt;
            if (robot.uTurnTimer > 0.60f && (robot.sensorStates[2] || (robot.uTurnTimer > 0.75f && (robot.sensorStates[1] || robot.sensorStates[3])))) {
                robot.isDoingUTurn = false;
                robot.uTurnTimer = 0.0f;
                termLogs.push_back("[MANEVRA] U-donusu tamamlandi, ana hatta cikiliyor.");
            }
        }

        if (robot.isRunning && (robot.hasMission || robot.isReturningHome) && !robot.obstacleDetected && robot.taskWaitTimer == 0.0f && !robot.isWaitingForBox && !robot.isDoingUTurn) {
            float steer = 0.0f;
            bool isPivotingInPlace = false;
            bool branchingToPocket = false;

            if (hasActiveTarget && !robot.isReturningHome && robot.branchCooldown <= 0.0f && distToActiveTarget < 110.0f) {
                if (angleDiffToTarget < -0.45f && angleDiffToTarget > -2.35f && (robot.leftSeesLine || robot.leftSeesTarget)) {
                    steer = -6.8f;
                    robot.lastTurnDir = -1;
                    branchingToPocket = true;
                } else if (angleDiffToTarget > 0.45f && angleDiffToTarget < 2.35f && (robot.rightSeesLine || robot.rightSeesTarget)) {
                    steer = 6.8f;
                    robot.lastTurnDir = 1;
                    branchingToPocket = true;
                }
            }

            if (!branchingToPocket) {
                if (activeCount > 0) {
                    float error = weightedSum / (float)activeCount;
                    steer = error * 5.2f + robot.frontLineAngleOffset * 2.0f;

                    if (error < -0.25f) robot.lastTurnDir = -1;
                    else if (error > 0.25f) robot.lastTurnDir = 1;

                    if (distToActiveTarget < 65.0f && fabsf(angleDiffToTarget) < 0.6f) {
                        steer += angleDiffToTarget * 3.5f;
                    }
                } else {
                    isPivotingInPlace = true;

                    if (robot.leftSeesLine && !robot.rightSeesLine) {
                        steer = -6.8f;
                        robot.lastTurnDir = -1;
                    } else if (robot.rightSeesLine && !robot.leftSeesLine) {
                        steer = 6.8f;
                        robot.lastTurnDir = 1;
                    } else if (robot.leftSeesLine && robot.rightSeesLine) {
                        if (angleDiffToTarget < 0.0f) {
                            steer = -6.8f;
                            robot.lastTurnDir = -1;
                        } else {
                            steer = 6.8f;
                            robot.lastTurnDir = 1;
                        }
                    } else if (robot.frontSeesLine) {
                        steer = (robot.frontLineAngleOffset < 0.0f) ? -6.0f : 6.0f;
                    } else {
                        steer = (robot.lastTurnDir < 0) ? -6.5f : 6.5f;
                    }
                }
            }

            float jitter = isPivotingInPlace ? 0.0f : (((float)(rand() % 100 - 50) / 1000.0f) * 0.08f);
            robot.angle = NormalizeAngle(robot.angle + (steer + jitter) * dt);

            if (isPivotingInPlace) {
                robot.speed = 0.0f;
                sf::Vector2f snapped = SnapToNearestTrack(robot.pos, trackGrid, simW, simH, 50);
                robot.pos += (snapped - robot.pos) * std::min(1.0f, dt * 12.0f);
            } else {
                float turnSlowdown = std::clamp(1.0f - (fabsf(steer) / 7.5f), 0.30f, 1.0f);
                float targetSpeed = robot.baseSpeed * turnSlowdown;

                if (branchingToPocket || distToActiveTarget < 70.0f) {
                    targetSpeed = std::min(targetSpeed, 42.0f);
                }
                if (robot.frontWarning) {
                    targetSpeed = std::min(targetSpeed, 45.0f);
                }

                robot.speed = targetSpeed;
                robot.pos += sf::Vector2f(cosf(robot.angle), sinf(robot.angle)) * robot.speed * dt;

                sf::Vector2f snapped = SnapToNearestTrack(robot.pos, trackGrid, simW, simH, 24);
                robot.pos += (snapped - robot.pos) * std::min(1.0f, dt * 5.0f);
            }

            for (size_t bi = 0; bi < robot.carriedBoxIds.size(); ++bi) {
                int bId = robot.carriedBoxIds[bi];
                for (auto& bx : boxes) {
                    if (bx.id == bId) {
                        float offsetDist = 14.0f + (float)bi * 16.0f;
                        bx.pos = robot.pos - forward * offsetDist;
                        break;
                    }
                }
            }
        } else {
            robot.speed = 0.0f;
        }

        battery.Update(dt, robot.speed > 0.0f, robot.payloadKg);

        // RENDER: 1. PENCERE
        simWin.clear(sf::Color(245, 245, 245));
        sf::Sprite canvasSprite(canvas.getTexture());
        simWin.draw(canvasSprite);

        if (isMouseDown && shiftHeld && simWin.hasFocus()) {
            sf::Vector2f curMouse = simWin.mapPixelToCoords(sf::Mouse::getPosition(simWin));
            float dx = curMouse.x - strokeStartPos.x;
            float dy = curMouse.y - strokeStartPos.y;
            sf::Vector2f snappedEnd = (fabsf(dx) >= fabsf(dy)) ? sf::Vector2f(curMouse.x, strokeStartPos.y)
                                                               : sf::Vector2f(strokeStartPos.x, curMouse.y);

            float lineLen = std::hypot(snappedEnd.x - strokeStartPos.x, snappedEnd.y - strokeStartPos.y);
            float lineAngle = atan2f(snappedEnd.y - strokeStartPos.y, snappedEnd.x - strokeStartPos.x) * 180.0f / PI;

            sf::RectangleShape previewRect(sf::Vector2f(lineLen, brushRadius * 2.0f));
            previewRect.setOrigin(0, brushRadius);
            previewRect.setPosition(strokeStartPos);
            previewRect.setRotation(lineAngle);
            previewRect.setFillColor(sf::Color(20, 20, 20, 180));
            simWin.draw(previewRect);

            sf::CircleShape p1Dot(brushRadius), p2Dot(brushRadius);
            p1Dot.setOrigin(brushRadius, brushRadius); p1Dot.setPosition(strokeStartPos); p1Dot.setFillColor(sf::Color(20, 20, 20, 180));
            p2Dot.setOrigin(brushRadius, brushRadius); p2Dot.setPosition(snappedEnd); p2Dot.setFillColor(sf::Color(20, 20, 20, 180));
            simWin.draw(p1Dot); simWin.draw(p2Dot);

            sf::Vertex guideLine[] = {
                sf::Vertex(strokeStartPos, sf::Color(0, 150, 255, 200)),
                sf::Vertex(snappedEnd, sf::Color(0, 150, 255, 200))
            };
            simWin.draw(guideLine, 2, sf::Lines);
        }

        sf::RectangleShape homeRect(sf::Vector2f(robot.length + 8.0f, robot.width + 8.0f));
        homeRect.setOrigin((robot.length + 8.0f) / 2.0f, (robot.width + 8.0f) / 2.0f);
        homeRect.setPosition(robot.homePos);
        homeRect.setFillColor(robot.isReturningHome ? sf::Color(100, 255, 100, 95) : sf::Color(200, 200, 200, 75));
        homeRect.setOutlineThickness(2.0f);
        homeRect.setOutlineColor(robot.isReturningHome ? sf::Color(40, 190, 40) : sf::Color(130, 130, 130));
        simWin.draw(homeRect);

        if (fontOk) {
            sf::Text ht("HOME", font, 11);
            ht.setOrigin(15, 6);
            ht.setPosition(robot.homePos);
            ht.setFillColor(sf::Color(60, 60, 60));
            simWin.draw(ht);
        }

        for (const auto& st : stations) {
            bool isCurrentTarget = false;
            if (robot.hasMission && !robot.isReturningHome && robot.currentTaskIdx < robot.taskQueue.size()) {
                if (robot.taskQueue[robot.currentTaskIdx].stationId == st.id) isCurrentTarget = true;
            }

            sf::CircleShape cs(st.radius);
            cs.setOrigin(st.radius, st.radius);
            cs.setPosition(st.pos);
            cs.setFillColor(isCurrentTarget ? sf::Color(255, 215, 0, 140) : sf::Color(100, 180, 255, 100));
            cs.setOutlineThickness(isCurrentTarget ? 3.0f : 2.0f);
            cs.setOutlineColor(isCurrentTarget ? sf::Color(220, 120, 0) : sf::Color(30, 100, 220));
            simWin.draw(cs);

            if (fontOk) {
                sf::Text t("T" + std::to_string(st.id), font, 16);
                t.setOrigin(8, 10);
                t.setPosition(st.pos);
                t.setFillColor(sf::Color(10, 50, 150));
                simWin.draw(t);
            }
        }

        for (const auto& bx : boxes) {
            sf::RectangleShape rs(sf::Vector2f(bx.size, bx.size));
            rs.setOrigin(bx.size / 2.0f, bx.size / 2.0f);
            rs.setPosition(bx.pos);
            rs.setFillColor(sf::Color(240, 130, 40));
            rs.setOutlineThickness(2.0f);
            rs.setOutlineColor(sf::Color(140, 60, 10));
            simWin.draw(rs);

            if (fontOk) {
                sf::Text t("K" + std::to_string(bx.id), font, 14);
                t.setOrigin(7, 9);
                t.setPosition(bx.pos);
                t.setFillColor(sf::Color::Black);
                simWin.draw(t);
            }
        }

        for (const auto& obs : obstacles) {
            bool onLine = IsObstacleOnTrack(obs, trackGrid, simW, simH);
            sf::CircleShape os(obs.radius);
            os.setOrigin(obs.radius, obs.radius);
            os.setPosition(obs.pos);
            os.setFillColor(onLine ? sf::Color(220, 50, 50) : sf::Color(180, 90, 90, 160));
            os.setOutlineThickness(2.5f);
            os.setOutlineColor(onLine ? sf::Color(140, 20, 20) : sf::Color(120, 120, 120));
            simWin.draw(os);

            if (fontOk) {
                sf::Text t("ENGEL", font, 10);
                t.setOrigin(16, 6);
                t.setPosition(obs.pos);
                t.setFillColor(sf::Color::White);
                simWin.draw(t);
            }
        }

        sf::Color leftConeColor = robot.leftSeesLine ? sf::Color(0, 255, 220, 85) : sf::Color(0, 210, 255, 35);
        DrawCameraCone(simWin, leftCamOrigin, leftAngle, sideFov, sideRange, robot.leftObstacle, false, robot.leftSeesTarget, leftConeColor, false);

        sf::Color rightConeColor = robot.rightSeesLine ? sf::Color(0, 255, 220, 85) : sf::Color(0, 210, 255, 35);
        DrawCameraCone(simWin, rightCamOrigin, rightAngle, sideFov, sideRange, robot.rightObstacle, false, robot.rightSeesTarget, rightConeColor, false);

        DrawCameraCone(simWin, frontCamOrigin, robot.angle, frontFov, frontRange, robot.frontObstacle, robot.frontWarning, robot.frontSeesTarget, sf::Color(50, 255, 80, 45), true);

        sf::RectangleShape rBody(sf::Vector2f(robot.length, robot.width));
        rBody.setOrigin(robot.length / 2.0f, robot.width / 2.0f);
        rBody.setPosition(robot.pos);
        rBody.setRotation(robot.angle * 180.0f / PI);
        rBody.setFillColor(sf::Color(45, 52, 54));
        rBody.setOutlineThickness(2.0f);
        rBody.setOutlineColor(sf::Color(180, 180, 180));
        simWin.draw(rBody);

        for (int i = 0; i < 5; ++i) {
            sf::CircleShape ss(3.5f);
            ss.setOrigin(3.5f, 3.5f);
            ss.setPosition(robot.sensorPositions[i]);
            ss.setFillColor(robot.sensorStates[i] ? sf::Color(50, 255, 50) : sf::Color(255, 40, 40));
            simWin.draw(ss);
        }

        if (fontOk) {
            sf::RectangleShape bar(sf::Vector2f(simW, 30));
            bar.setPosition(0, simH - 30);
            bar.setFillColor(sf::Color(20, 20, 20, 200));
            simWin.draw(bar);

            const char* tipStr = shiftHeld ? "[CETVEL MODU AKTIF] Fareyi surukleyip birakarak tam 90 deg duz cizgi cizin." :
                                             "SHIFT + Surukle: 90 deg Cetvel | Sol Tik: Ciz | Sag Tik: Engel | T: Istasyon | K: Kutu | R: Home Kur";
            sf::Color tipCol = shiftHeld ? sf::Color(100, 220, 255) : sf::Color::White;
            sf::Text tip(tipStr, font, 13);
            tip.setPosition(15, simH - 24);
            tip.setFillColor(tipCol);
            simWin.draw(tip);
        }
        simWin.display();

        // RENDER: 2. PENCERE
        dataWin.clear(sf::Color(26, 30, 38));

        if (fontOk) {
            sf::RectangleShape hBar(sf::Vector2f(dashW, 45));
            hBar.setFillColor(sf::Color(15, 17, 22));
            dataWin.draw(hBar);

            sf::Text title("ROBOT TELEMETRI VE SENSORLER", font, 16);
            title.setPosition(15, 12);
            title.setFillColor(sf::Color::White);
            dataWin.draw(title);

            int y = 58;

            const char* stStr = robot.obstacleDetected ? "ENGEL DURUSU (%50 MENZIL)" :
                                (robot.isDoingUTurn ? "CEP HATTI U-DONUSU" :
                                (robot.isReturningHome ? "BASLANGICA DONUS (HOME)" :
                                (robot.isWaitingForBox ? "KUTU BEKLENIYOR (K BASIN)" :
                                (robot.taskWaitTimer > 0.0f ? "MANEVRA / YUK AKTARIMI" :
                                (robot.isRunning ? "OTONOM GOREVDE" : "PARK EDILDI (IDLE)")))));

            sf::Color stCol = robot.obstacleDetected ? sf::Color(255, 80, 80) :
                              (robot.isDoingUTurn ? sf::Color(180, 100, 255) :
                              (robot.isReturningHome ? sf::Color(100, 255, 180) :
                              (robot.isWaitingForBox ? sf::Color(255, 140, 0) :
                              (robot.taskWaitTimer > 0.0f ? sf::Color(255, 200, 50) :
                              (robot.isRunning ? sf::Color(80, 255, 100) : sf::Color(200, 200, 200))))));

            sf::Text lblMode("Calisma Modu:", font, 13); lblMode.setPosition(20, y); lblMode.setFillColor(sf::Color(160, 160, 160)); dataWin.draw(lblMode);
            y += 20;
            sf::RectangleShape mBox(sf::Vector2f(dashW - 40, 32)); mBox.setPosition(20, y); mBox.setFillColor(sf::Color(35, 40, 50)); mBox.setOutlineThickness(1.5f); mBox.setOutlineColor(stCol); dataWin.draw(mBox);
            sf::Text txtMode(stStr, font, 14); txtMode.setPosition(30, y + 6); txtMode.setFillColor(stCol); dataWin.draw(txtMode);

            y += 46;
            sf::RectangleShape camBox(sf::Vector2f(dashW - 40, 42)); camBox.setPosition(20, y); camBox.setFillColor(sf::Color(35, 40, 50)); dataWin.draw(camBox);
            
            std::string fStr = robot.frontObstacle ? "ON: DUR(%50)" : (robot.frontWarning ? "ON: YAKINLAS" : (robot.frontSeesTarget ? "ON: HEDEF" : "ON: ACIK"));
            sf::Color fCol = robot.frontObstacle ? sf::Color(255, 80, 80) : (robot.frontWarning ? sf::Color(255, 165, 0) : (robot.frontSeesTarget ? sf::Color(255, 215, 0) : sf::Color(80, 255, 100)));
            sf::Text fCam(fStr, font, 12); fCam.setPosition(28, y + 12); fCam.setFillColor(fCol); dataWin.draw(fCam);

            std::string lStr = robot.leftObstacle ? "SOL: ENGEL" : (robot.leftSeesTarget ? "SOL: HEDEF" : (robot.leftSeesLine ? "SOL: CIZGI" : "SOL: ACIK"));
            sf::Color lCol = robot.leftObstacle ? sf::Color(255, 80, 80) : (robot.leftSeesTarget ? sf::Color(255, 215, 0) : (robot.leftSeesLine ? sf::Color(0, 255, 220) : sf::Color(0, 210, 255)));
            sf::Text lCam(lStr, font, 12); lCam.setPosition(165, y + 12); lCam.setFillColor(lCol); dataWin.draw(lCam);

            std::string rStr = robot.rightObstacle ? "SAG: ENGEL" : (robot.rightSeesTarget ? "SAG: HEDEF" : (robot.rightSeesLine ? "SAG: CIZGI" : "SAG: ACIK"));
            sf::Color rCol = robot.rightObstacle ? sf::Color(255, 80, 80) : (robot.rightSeesTarget ? sf::Color(255, 215, 0) : (robot.rightSeesLine ? sf::Color(0, 255, 220) : sf::Color(0, 210, 255)));
            sf::Text rCam(rStr, font, 12); rCam.setPosition(305, y + 12); rCam.setFillColor(rCol); dataWin.draw(rCam);

            y += 50;
            float degAngle = fmodf(robot.angle * 180.0f / PI, 360.0f);
            if (degAngle < 0) degAngle += 360.0f;
            sf::RectangleShape aBox(sf::Vector2f(dashW - 40, 38)); aBox.setPosition(20, y); aBox.setFillColor(sf::Color(35, 40, 50)); dataWin.draw(aBox);
            sf::Text aLbl("Robot Yon Acisi:", font, 14); aLbl.setPosition(30, y + 9); aLbl.setFillColor(sf::Color::White); dataWin.draw(aLbl);
            sf::Text aVal(std::to_string((int)degAngle) + "." + std::to_string((int)(degAngle * 10) % 10) + " deg", font, 16); aVal.setPosition(dashW - 140, y + 7); aVal.setFillColor(sf::Color(100, 200, 255)); dataWin.draw(aVal);

            y += 46;
            sf::RectangleShape cBox(sf::Vector2f(dashW - 40, 38)); cBox.setPosition(20, y); cBox.setFillColor(sf::Color(35, 40, 50)); dataWin.draw(cBox);
            sf::Text cLbl("Otonom Gorus Guveni:", font, 14); cLbl.setPosition(30, y + 9); cLbl.setFillColor(sf::Color::White); dataWin.draw(cLbl);
            sf::Color cCol = (robot.visionConfidence > 85.0f) ? sf::Color(100, 255, 100) : sf::Color(255, 200, 50);
            sf::Text cVal("% " + std::to_string((int)robot.visionConfidence) + "." + std::to_string((int)(robot.visionConfidence * 10) % 10), font, 16); cVal.setPosition(dashW - 140, y + 7); cVal.setFillColor(cCol); dataWin.draw(cVal);

            y += 46;
            sf::RectangleShape lBox(sf::Vector2f(dashW - 40, 38)); lBox.setPosition(20, y); lBox.setFillColor(sf::Color(35, 40, 50)); dataWin.draw(lBox);
            sf::Text lLbl("Kantar Yuku (Load):", font, 14); lLbl.setPosition(30, y + 9); lLbl.setFillColor(sf::Color::White); dataWin.draw(lLbl);
            sf::Text lVal(std::to_string((int)robot.payloadKg) + ".0 kg", font, 16); lVal.setPosition(dashW - 140, y + 7); lVal.setFillColor(robot.payloadKg > 0.0f ? sf::Color(255, 160, 50) : sf::Color(160, 160, 160)); dataWin.draw(lVal);

            y += 46;
            sf::RectangleShape bBox(sf::Vector2f(dashW - 40, 52)); bBox.setPosition(20, y); bBox.setFillColor(sf::Color(35, 40, 50)); dataWin.draw(bBox);
            sf::Text bLbl("Batarya BMS (ADC 1Hz Ornekleme):", font, 12); bLbl.setPosition(30, y + 5); bLbl.setFillColor(sf::Color(180, 180, 180)); dataWin.draw(bLbl);
            std::string bStr = "% " + std::to_string((int)battery.sampledPercentage) + "." + std::to_string((int)(battery.sampledPercentage*10)%10) + "  (" + std::to_string((int)battery.sampledVoltage) + "." + std::to_string((int)(battery.sampledVoltage*100)%100) + " V)";
            sf::Text bVal(bStr, font, 15); bVal.setPosition(30, y + 24); bVal.setFillColor(sf::Color(100, 255, 100)); dataWin.draw(bVal);
            sf::Text aAmps(std::to_string((int)battery.currentDraw_A) + "." + std::to_string((int)(battery.currentDraw_A*100)%100) + " A", font, 14); aAmps.setPosition(dashW - 110, y + 24); aAmps.setFillColor(sf::Color(255, 210, 80)); dataWin.draw(aAmps);

            y += 66;
            int totalTasks = (int)robot.taskQueue.size();
            int curStep = (int)robot.currentTaskIdx + 1;
            if (curStep > totalTasks) curStep = totalTasks;

            std::string qHeader = "GOREV PLANI " + (totalTasks > 0 ? ("[" + std::to_string(curStep) + " / " + std::to_string(totalTasks) + "]") : "");
            sf::Text qLbl(qHeader, font, 13); qLbl.setPosition(20, y); qLbl.setFillColor(sf::Color(160, 160, 160)); dataWin.draw(qLbl);
            y += 20;

            sf::RectangleShape qBox(sf::Vector2f(dashW - 40, 230)); qBox.setPosition(20, y); qBox.setFillColor(sf::Color(18, 20, 26)); qBox.setOutlineThickness(1.0f); qBox.setOutlineColor(sf::Color(60, 65, 80)); dataWin.draw(qBox);

            if (robot.taskQueue.empty() && !robot.isReturningHome) {
                sf::Text noT("Gorev yok. Terminalden giriniz:\nOrnek: t1k t3b / t2k t4b", font, 13);
                noT.setPosition(35, y + 25); noT.setFillColor(sf::Color(120, 120, 120)); dataWin.draw(noT);
            } else {
                int visibleCount = 6;
                int startIdx = std::max(0, (int)robot.currentTaskIdx - 2);
                if (startIdx + visibleCount > totalTasks) startIdx = std::max(0, totalTasks - visibleCount);
                int endIdx = std::min(totalTasks, startIdx + visibleCount);

                int ty = y + 12;
                for (int i = startIdx; i < endIdx; ++i) {
                    bool cur = (i == (int)robot.currentTaskIdx && robot.hasMission && !robot.isReturningHome);
                    bool done = (i < (int)robot.currentTaskIdx);
                    std::string act = (robot.taskQueue[i].action == 'k') ? "KUTU AL" : "KUTU BIRAK";
                    std::string line = (cur ? ">> " : (done ? "[OK] " : "     ")) + std::to_string(i + 1) + ". Adim: T" + std::to_string(robot.taskQueue[i].stationId) + " -> " + act;

                    sf::Text tLine(line, font, 13);
                    tLine.setPosition(30, ty);
                    tLine.setFillColor(cur ? (robot.isWaitingForBox ? sf::Color(255, 140, 0) : sf::Color(255, 215, 0)) : (done ? sf::Color(90, 90, 90) : sf::Color(200, 200, 200)));
                    dataWin.draw(tLine);
                    ty += 24;
                }

                if (robot.isReturningHome) {
                    sf::Text homeStep(">> SON ADIM: HOME (PARK ALANI)", font, 13);
                    homeStep.setPosition(30, ty);
                    homeStep.setFillColor(sf::Color(100, 255, 180));
                    dataWin.draw(homeStep);
                }
            }
        }
        dataWin.display();

        // RENDER: 3. PENCERE
        termWin.clear(sf::Color(10, 12, 16));

        if (fontOk) {
            sf::RectangleShape tBar(sf::Vector2f(termW, 30));
            tBar.setFillColor(sf::Color(20, 24, 32));
            termWin.draw(tBar);

            sf::Text tTitle("TEKNOFEST WMS KOMUT VE LOG TERMINALI", font, 12);
            tTitle.setPosition(12, 7);
            tTitle.setFillColor(sf::Color(180, 180, 180));
            termWin.draw(tTitle);

            int ly = 42;
            int startIdx = std::max(0, (int)termLogs.size() - 11);
            for (size_t i = startIdx; i < termLogs.size(); ++i) {
                sf::Text logLine(termLogs[i], font, 13);
                logLine.setPosition(15, ly);
                logLine.setFillColor(termLogs[i].find("[BEKLEME]") != std::string::npos ? sf::Color(255, 140, 0) :
                                     (termLogs[i].find("[CETVEL]") != std::string::npos ? sf::Color(100, 220, 255) :
                                     (termLogs[i].find("[MANEVRA]") != std::string::npos ? sf::Color(180, 100, 255) :
                                     (termLogs[i].find("[HATA]") != std::string::npos ? sf::Color(255, 80, 80) :
                                     (termLogs[i].find("AGV-WMS>") != std::string::npos ? sf::Color(255, 230, 80) :
                                     (termLogs[i].find("[PLAN]") != std::string::npos ? sf::Color(100, 200, 255) : sf::Color(100, 255, 120)))))));
                termWin.draw(logLine);
                ly += 22;
            }

            sf::RectangleShape inBox(sf::Vector2f(termW - 20, 36));
            inBox.setPosition(10, termH - 46);
            inBox.setFillColor(sf::Color(20, 25, 35));
            inBox.setOutlineThickness(1.5f);
            inBox.setOutlineColor(sf::Color(50, 180, 255));
            termWin.draw(inBox);

            std::string promptStr = "AGV-WMS> " + termInput + (showCursor ? "_" : " ");
            sf::Text pTxt(promptStr, font, 14);
            pTxt.setPosition(18, termH - 38);
            pTxt.setFillColor(sf::Color::Cyan);
            termWin.draw(pTxt);
        }
        termWin.display();
    }

    return 0;
}
