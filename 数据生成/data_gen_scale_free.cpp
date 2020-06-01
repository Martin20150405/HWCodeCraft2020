#include <bits/stdc++.h>

using namespace std;

int NE = 2000000;
int ei = 20000;
int NG = NE / ei;
//alpha表示按照入度分布连边（入度占总入度比越高的点越容易被选中）
//beta表示按照出度分布
//gamma表示按照入度和出度分布
//[0,alpha] ->alpha
//(alpha,alpha+beta] -> beta
//(alpha+beta,1] ->gamma
//例：结点N=100，边E=500，则alpha+beta=0.2，gamma=0.8
double alpha = 0.32, beta = 0.32, gamma_ = 1.0 - alpha - beta;
enum {
    ALPHA, BETA, GAMMA
};

// random number generater
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
uniform_int_distribution<int> distributionW(1, 0x7fffffff);
uniform_int_distribution<int> distributionID(0, 0x7fffffff);

//no self-loop or same edge
map<pair<int, int>, int> vis;

vector<pair<pair<int, int>, int>> E;

int currentId = 0;

//前一个图的最后加入点有alpha+beta几率和后一个图连接
void generateSubGraph(int st) {
    printf("Generating SubGraph %d\n", st);
    srand(time(NULL));

    map<int, int> inDegrees;
    map<int, int> outDegrees;

    auto add_edge = [&](int x, int y, int c) -> bool {
        if (x == y || vis.count(make_pair(x, y))) return false;
        vis[make_pair(x, y)] = c;
        E.push_back(make_pair(make_pair(x, y), c));
        outDegrees[x]++;
        inDegrees[y]++;
//        printf("Adding %d-%d[%d]\n",x,y,c);
        return true;
    };
    int e = ei;
    add_edge(335201314 + st, 442911314 + st, distributionW(rng));
    e -= 1;

    auto get_task_state = []() {
        double r = rand() * 1.0 / RAND_MAX;
        if (r > alpha + beta) return GAMMA;
        else if (r > alpha) return BETA;
        else return ALPHA;
    };

    auto get_selected_idx = [&](map<int, int> &degrees) -> int {
        vector<pair<int, double>> degreeSum;
        double sum = 0;
        for (auto x:degrees) {
            sum += x.second;
            degreeSum.emplace_back(x.first, sum);
        }
        for (auto &p:degreeSum) p.second /= sum;
        double x = rand() * 1.0 / RAND_MAX;
        for (auto &p:degreeSum) {
            if (x < p.second) {
//                printf("%lf/%lf\n",x,p.second);
                return p.first;
            }
        }
        return -1;
    };

    while (e) {
        //假定按照入度选中的点为v，按照出度选中的点为u，新结点为x，边权重为w，
        //则每次有alpha概率产生[x,v,w]，有beta概率产生[u,x,w]，有gamma概率产生[u,v,w]
        auto state = get_task_state();
//        cout<<"State "<< state<<endl;
        int u = get_selected_idx(inDegrees);
        int v = get_selected_idx(outDegrees);
        int x = currentId;
        int w = distributionW(rng);
        if (state == ALPHA) {
            if (v < 0) continue;
            while (!add_edge(x, v, w)) v = get_selected_idx(outDegrees);
            e -= 1;
            currentId++;
        } else if (state == BETA) {
            if (u < 0) continue;
            while (!add_edge(u, x, w)) u = get_selected_idx(inDegrees);
            e -= 1;
            currentId++;
        } else if (state == GAMMA) {
            double x = rand() * 1.0 / RAND_MAX;
            //仅仅是为了减小环的数量，随机丢弃50%旧点连接
            //会导致最终产生的结点数超出预期
            if (x < 0.5) continue;
            if (u < 0 || v < 0) continue;
            bool ret;
            while (!(ret = add_edge(u, v, w))) {
                u = get_selected_idx(inDegrees), v = get_selected_idx(outDegrees);
                if (u < 0 || v < 0) break;
            }
            if (ret)
                e -= 1;
        }
    }
//    for(auto x:inDegrees){
//        printf("[IN] %d %d\n",x.first,x.second);
//    }
//    for(auto x:outDegrees){
//        printf("[OUT] %d %d\n",x.first,x.second);
//    }
}

int main() {

    for (int i = 0; i < NG; ++i) {
        generateSubGraph(i);
    }

    random_shuffle(E.begin(), E.end());

    ///remap inputs
    unordered_set<int> originID;
    map<int, int> remap;
    unordered_set<int> remmapedID;
    for (auto e: E) {
        auto u = e.first.first;
        auto v = e.first.second;
        originID.insert(u);
        originID.insert(v);
    }
    for (int x:originID) {
        int y = distributionID(rng);
        while (remmapedID.find(y) != remmapedID.end())
            y = distributionID(rng);
        remmapedID.insert(y);
        remap[x] = y;
    }

    printf("Total Nodes: %d\n", (int) remmapedID.size());
    printf("Total Edges: %d\n", (int) E.size());

    ///output
    FILE *fp = fopen("test_data_SFN.txt", "wb");
    for (auto e: E) {
        auto u = to_string(remap[e.first.first]);
        auto v = to_string(remap[e.first.second]);
        auto c = to_string(e.second);
        fwrite(u.c_str(), u.size(), sizeof(char), fp);
        fwrite(",", 1, sizeof(char), fp);
        fwrite(v.c_str(), v.size(), sizeof(char), fp);
        fwrite(",", 1, sizeof(char), fp);
        fwrite(c.c_str(), c.size(), sizeof(char), fp);
        fwrite("\n", 1, sizeof(char), fp);
    }
    return 0;
}