#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

using namespace std;

struct Node {
    int x, y;
    double cost;
    bool operator<(const Node& other) const {
        return cost > other.cost;
    }
};

double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool isValid(int x, int y, int rows, int cols, const vector<vector<int>>& grid) {
    return x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] == 0;
}

void printGrid(const vector<vector<int>>& grid, int startX, int startY, int endX, int endY) {
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[0].size(); j++) {
            if (i == startX && j == startY) {
                cout << "S ";
            } else if (i == endX && j == endY) {
                cout << "E ";
            } else if (grid[i][j] == 1) {
                cout << "# ";
            } else {
                cout << ". ";
            }
        }
        cout << endl;
    }
    cout << endl;
}

void astar(int startX, int startY, int endX, int endY, const vector<vector<int>>& grid) {
    int rows = grid.size(), cols = grid[0].size();
    vector<vector<double>> dist(rows, vector<double>(cols, numeric_limits<double>::infinity()));
    priority_queue<Node> pq;
    pq.push({startX, startY, 0});
    dist[startX][startY] = 0;

    cout << "Grid for A*: \n";
    printGrid(grid, startX, startY, endX, endY);

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.x == endX && current.y == endY) {
            cout << "Destination reached by A*\n";
            return;
        }

        for (const auto& d : vector<pair<int, int>>{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}) {
            int nx = current.x + d.first;
            int ny = current.y + d.second;

            if (isValid(nx, ny, rows, cols, grid)) {
                double newCost = dist[current.x][current.y] + 1;
                double fCost = newCost + heuristic(nx, ny, endX, endY);

                if (newCost < dist[nx][ny]) {
                    dist[nx][ny] = newCost;
                    pq.push({nx, ny, fCost});
                }
            }
        }
    }

    cout << "Destination cannot be reached by A*\n";
}

void bestFirstSearch(int startX, int startY, int endX, int endY, const vector<vector<int>>& grid) {
    int rows = grid.size(), cols = grid[0].size();
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    priority_queue<Node> pq;
    pq.push({startX, startY, heuristic(startX, startY, endX, endY)});

    cout << "Grid for Best First Search: \n";
    printGrid(grid, startX, startY, endX, endY);

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.x == endX && current.y == endY) {
            cout << "Destination reached by Best First Search\n";
            return;
        }

        if (visited[current.x][current.y]) continue;
        visited[current.x][current.y] = true;

        for (const auto& d : vector<pair<int, int>>{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}) {
            int nx = current.x + d.first;
            int ny = current.y + d.second;

            if (isValid(nx, ny, rows, cols, grid) && !visited[nx][ny]) {
                pq.push({nx, ny, heuristic(nx, ny, endX, endY)});
            }
        }
    }

    cout << "Destination cannot be reached by Best First Search\n";
}

int main() {
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {1, 1, 0, 0, 0},
        {0, 0, 0, 1, 0}
    };

    int startX = 0, startY = 0, endX = 4, endY = 4;

    astar(startX, startY, endX, endY, grid);
    bestFirstSearch(startX, startY, endX, endY, grid);

    return 0;
}
