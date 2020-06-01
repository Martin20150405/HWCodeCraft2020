#include <bits/stdc++.h>

using namespace std;

int e = 2000000;
int v = 111313;

// random number generater
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
// generate integer in range [1, V]
uniform_int_distribution<int> distributionUV(0, v);
uniform_int_distribution<int> distributionW(1, 0x7fffffff);

int main() {
    srand((unsigned long long) new char);
    map<pair<int, int>,int> vis;
    vector<pair<pair<int, int>,int>> E;
    auto add_edge = [&](int x, int y,int c) -> bool {
        if (x == y || vis.count(make_pair(x, y))) return false;
        vis[make_pair(x, y)]=c;
        E.push_back(make_pair(make_pair(x, y),c));
        return true;
    };
    //K13
    for (int i = 6000; i <= 6000 + 12; i++) {
        for (int j = 6000; j <= 6000 + 12; j++) {
            if (i == j) continue;
            add_edge(i, j,rand());
            e -= 1;
        }
    }
    //K14
    for (int i = 10000; i <= 10000 + 13; i++) {
        for (int j = 10000; j <= 10000 + 13; j++) {
            if (i == j) continue;
            add_edge(i, j,rand());
            e -= 1;
        }
    }
    //K13
    for (int i = 25123; i <= 25123 + 12; i++) {
        for (int j = 25123; j <= 25123 + 12; j++) {
            if (i == j) continue;
            add_edge(i, j,rand());
            e -= 1;
        }
    }

    cout<<e<<endl;
    while (e) {
        int x = distributionUV(rng);
        int y = distributionUV(rng);
        int c = distributionW(rng);
        if (add_edge(x, y,c)) e -= 1;
    }
    random_shuffle(E.begin(), E.end());

    FILE *fp = fopen("test_data_200501.txt", "wb");

    for (auto e: E) {
        auto u=to_string(e.first.first);
        auto v=to_string(e.first.second);
        auto c=to_string(e.second);
        fwrite(u.c_str(), u.size(), sizeof(char), fp);
        fwrite(",", 1, sizeof(char), fp);
        fwrite(v.c_str(), v.size(), sizeof(char), fp);
        fwrite(",", 1, sizeof(char), fp);
        fwrite(c.c_str(), c.size(), sizeof(char), fp);
        fwrite("\n", 1, sizeof(char), fp);
    }
    return 0;
}