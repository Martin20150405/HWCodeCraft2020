#include <bits/stdc++.h>
#include <ext/pb_ds/hash_policy.hpp>
#include <ext/pb_ds/assoc_container.hpp>
#include <ext/pb_ds/priority_queue.hpp>
#include <fcntl.h>
#include <unistd.h>
//#include "robin_map.h"
//#include "lni_vector.h"
using namespace std;

///@Author 小白兔奶糖
///@Date 2020.5
///@Algorithm [Brandes-2001] + [Baglioni-2012]

#define CONFIG_USE_MMAP
#define TEST

#ifdef CONFIG_USE_MMAP
#include <sys/mman.h>
#include <sys/stat.h>
#endif

///Timer
#ifdef _WIN32   // windows

#include <sysinfoapi.h>

#else   // unix
#include <sys/time.h>
#endif

mutex printLock;

struct UniversalTimer {
    vector<pair<string, int>> logPairs;

    UniversalTimer() {
        setTime();
    }

#ifdef _WIN32
    unsigned int startTime;
    unsigned int endTime;
#else
    struct timeval startTime;
    struct timeval endTime;
#endif

    void setTime() {
#ifdef _WIN32
        startTime = GetTickCount();
#else
        gettimeofday(&startTime, NULL);
#endif
    }

    int getElapsedTimeMS() {
#ifdef _WIN32
        endTime = GetTickCount();
        return int(endTime - startTime);
#else
        gettimeofday(&endTime, NULL);
        return int(1000 * (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec) / 1000);
#endif
    }

    void logTimeImpl(string tag, int elapsedTimeMS) {
        printLock.lock();
        printf("Time consumed for %-20s is %d ms.\n", tag.c_str(), elapsedTimeMS);
        printLock.unlock();
    }

    void logTime(string tag) {
        logTimeImpl(tag, getElapsedTimeMS());
    }

    void resetTimeWithTag(string tag) {
        logPairs.emplace_back(make_pair(tag, getElapsedTimeMS()));
        setTime();
    }

    void printLogs() {
        for (auto &x:logPairs)
            logTimeImpl(x.first, x.second);
    }

    inline void updateProgress(int cur,int tot,bool endFlag){
        if(!endFlag && cur%100!=0) return;
        printLock.lock();
        char buf[64];
        sprintf(buf,"%d/%d (%.2lf%%) [%dms]",cur,tot,cur*100.0/tot,getElapsedTimeMS());
        cout<<"\r"<<buf;
        if(endFlag)
            cout<<endl;
        cout.flush();
        printLock.unlock();
    }

};

#ifdef TEST
UniversalTimer globalTimer;
#endif

typedef uint64_t ull;
typedef int64_t ll;
typedef uint32_t ui;
//两个点对之间最短路数量不会大于5000
typedef uint16_t us;
typedef double decimal;
#define mp(a, b) ((ull)a<<32 | b)
#define lp(a) ((a>>32) & 0xffffffff)
#define rp(a) (a & 0xffffffff)
#define clear_rp(a) (a ^ ((ui)a))

//MAXN won't exceed MAXE if only consider u
const int MAXE = 2500000 + 8;
const int MAXN = MAXE * 2;
const int MAX_SHORTEST_PATH_LEN_FOR_UINT16 = 50;
const int MAX_SHORTEST_PATH_LEN_FOR_UINT32 = 60;

#define NUM_THREAD 12
#define NUM_BLOCK 8192*5

template<class T>
struct Edge {
    ui to;
    T w;

    Edge() {}

    Edge(ui to, T w) : to(to), w(w) {}

    bool operator<(const Edge &rhs) const {
        return w < rhs.w;
    }
};

struct Node {
    //record G[st,ed) for Node i
    int st, ed;

    Node() {}

    Node(int st, int ed) : st(st), ed(ed) {}
};

//for scale free network : use stl:pq & I0O1
//if SCC<10 && I0O1 node < 10% ---> Graph is random
enum GRAPH_TYPE{
    SCALE_FREE=0,RANDOM=1
};

////Global data
///输入部分
//线上ID不连续
//id与权重使用ui表示
ui inputs[MAXE * 2]; //u-v pairs
ui weights[MAXE]; //w
__gnu_pbds::gp_hash_table<ui, ui> idHash; //remap sorted id to 1...n
ui tmpIds[MAXN]; //u buffer
ui ids[MAXN]; //sorted u

///预处理部分
bool weightFitUI32; //default false, set if maxWeight*MAX_SHORTEST_PATH_LEN<UINT32_MAX
bool weightFitUI16; //default false, set if maxWeight*MAX_SHORTEST_PATH_LEN<UINT16_MAX
ui maxWeight;

//特判入度为0，出度为1的情况
//对于此类点，将计算任务下放至后续结点
ui scaleFactor[MAXN];

///图表示部分
ui inDegreesStart[MAXN];
Edge<ui> edgesG[MAXE];
char graphType=SCALE_FREE;
//如果有大于5%的权值大于65535，则认为无法使用权值相关trick
bool weightOutOfRange;
//默认情况下，起止是0
Node G[MAXN]; //u:v-w

ui inDegrees[MAXN];
ui outDegrees[MAXN];

///图重构建临时变量
bool revisit[MAXN];
ui uiTmp[MAXN];
Node GTmp[MAXN]; //u:v-w
Edge<ui> edgesGTmp[MAXE];
ui old2NewId[MAXN]; //map oldId to newId
ui new2OldId[MAXN]; //map newId to oldId

///结果存储部分
decimal bcSum[MAXN];
int sortedBCIdx[MAXN];

///多线程同步部分
int blockNum;
int blockStride;
atomic_int currentBlock(0);
////

int nodeCnt = 0;
int inputSize = 0;
int inputPairSize = 0;
int edgeCntG = 0;

struct Sigma{
    int32_t pathCnt;
    //inDegreesSum indicate start idx in pList for i
    //pListEnd indicate end idx in pList for i
    int32_t pListEnd;
};

struct Bundle {
    int threadId;

    //Betweenness Centrality
    decimal bc[MAXN];
    //C_B(v)=sum(dep_sx[v])
    //dep_sx[v]=pathCnt_sx[v]/pathCnt_sx
    //即s-x最短路上对v的依赖
    decimal dependency[MAXN];
    //predecessor list
    //size won't exceed inDegrees
    ui pList[MAXN];

    // pathCnt<<32 | pListEnd
    Sigma sigma[MAXN];

    ui stack[MAXN];

    ui bucketCnt[128];
    ui bucketElement[128][25000];
};

static Bundle bundles[NUM_THREAD];

struct Tarjan{
    //在dfs时是第几个被搜到的（init=0)
    int dfn[MAXN];
    //这个点以及其子孙节点连的所有点中dfn最小的值
    int low[MAXN];
    //当前所有可能能构成强连通分量的点
    ui stack[MAXN];
    //sccId==0表示一个点在stack中
    //否则已经出栈
    int sccID[MAXN];
    int sccCnt;
    int nodeNum;
    int stackIdx;

    //map<int,int> sccIDCnt;

    void init(){
        sccCnt=stackIdx=nodeNum=0;
        memset(dfn,0,sizeof(int)*nodeCnt);
        memset(low,0,sizeof(int)*nodeCnt);
        memset(sccID,0,sizeof(int)*nodeCnt);
    }

    void findScc(ui u){
        low[u]=dfn[u]=++nodeNum;
        stack[++stackIdx]=u;
        int st = G[u].st;
        int lim = G[u].ed;
        for (int it = st; it < lim; ++it) {
            ui v = edgesG[it].to;
            if(!dfn[v]){
                findScc(v);
                low[u]=min(low[u],low[v]);
                //if low[v]>=dfn[u] -> 无法通过v及子树返回比u更小结点 -> u是割点
                //if low[v]>dfn[u] -> 无法通过v及子树返回u或u的祖先 -> <u,v>是桥
                //边双连通分量:同一边双内，点与点的边集中无桥
                //通过桥+染色可以求出边bcc
            }else if(!sccID[v]){ //在栈中
                low[u]=min(low[u],dfn[v]);
            }
        }
        if(low[u]==dfn[u]){
            //sccID[u]=++sccCnt;
            ++sccCnt;
            ui cur;
            //update sccID until reaching u
            do{
                cur=stack[stackIdx--];
                sccID[cur]=sccCnt;
                //sccIDCnt[sccCnt]++;
            }while(cur!=u);
        }
    }
};

struct DataLoader{
    Tarjan tarjan;
    int oorWeightCnt=0;

    void parseInput(string &testFile) {
#ifndef CONFIG_USE_MMAP
        FILE *file = fopen(testFile.c_str(), "r");
        ui u, v, w;
        while (fscanf(file, "%u,%u,%u", &u, &v, &w) != EOF) {
            //skip weight 0
            if(w==0) continue;
            inputs[inputSize++] = u;
            inputs[inputSize++] = v;
            weights[inputPairSize++] = w;
            if(w>maxWeight) maxWeight=w;
            if(w>65535) ++oorWeightCnt;
        }
#else
        int fd = open(testFile.c_str(), O_RDONLY);
        int fileLength = lseek(fd, 0, SEEK_END);
        char *buffer =
                (char *) mmap(NULL, fileLength, PROT_READ, MAP_PRIVATE, fd, 0);

        //as buffer reaching bufferEnd
        //value won't be in ['0','9'] (end of loop)
        while (*buffer >= '0') {
            ui u = 0, v = 0, w = 0;
            while (*buffer >= '0') {
                u = u * 10 + *(buffer++) - '0';
            }
            ++buffer;
            while (*buffer >= '0') {
                v = v * 10 + *(buffer++) - '0';
            }
            ++buffer;
            while (*buffer >= '0') {
                w = w * 10 + *(buffer++) - '0';
            }
            if (*buffer == '\r') ++buffer;
            ++buffer;
            //skip weight 0
            if(w==0) continue;
            inputs[inputSize++] = u;
            inputs[inputSize++] = v;
            weights[inputPairSize++] = w;
            if(w>maxWeight) maxWeight=w;
            if(w>65535) ++oorWeightCnt;
        }
#endif
        weightFitUI16=maxWeight<UINT16_MAX/MAX_SHORTEST_PATH_LEN_FOR_UINT16;
        weightFitUI32=maxWeight<UINT32_MAX/MAX_SHORTEST_PATH_LEN_FOR_UINT32;
        weightOutOfRange=oorWeightCnt*1.0/inputPairSize>0.05;
    }

    inline void sortInputs(int st, int ed) {
        sort(ids + st, ids + ed);
    }

    void merge2Result(int st1, int ed1, int st2, int ed2, ui *src, int dstSt, ui *dst) {
        //merge 2 sorted array
        int idx = 0, lim = (ed1 - st1) + (ed2 - st2);
        while (idx < lim) {
            ui mins = UINT32_MAX;
            bool flag = false;
            if (st1 < ed1 && src[st1] < mins) {
                mins = src[st1];
            }
            if (st2 < ed2 && src[st2] < mins) {
                flag = true;
                mins = src[st2];
            }
            dst[dstSt++] = mins;
            if (flag) ++st2;
            else ++st1;
            ++idx;
        }
    }

    inline void threadingSortInputs(int cnt) {
        //四路排序
        int segmentSize = cnt >> 2;
        int st[4] = {0, segmentSize, segmentSize * 2, segmentSize * 3};
        int ed[4] = {segmentSize, segmentSize * 2, segmentSize * 3, cnt};

        auto t1 = thread(&DataLoader::sortInputs,this, st[0], ed[0]);
        auto t2 = thread(&DataLoader::sortInputs,this, st[1], ed[1]);
        auto t3 = thread(&DataLoader::sortInputs,this, st[2], ed[2]);
        auto t4 = thread(&DataLoader::sortInputs,this, st[3], ed[3]);
        t1.join();
        t2.join();
        t3.join();
        t4.join();

        //4归并至2
        auto t5 = thread(&DataLoader::merge2Result,this, st[0], ed[0], st[1], ed[1], ids, st[0], tmpIds);
        auto t6 = thread(&DataLoader::merge2Result,this, st[2], ed[2], st[3], ed[3], ids, st[2], tmpIds);
        t5.join();
        t6.join();
        //2归并至1
        merge2Result(st[0], ed[1], st[2], ed[3], tmpIds, st[0], ids);
    }

    void constructGraph() {
#ifdef TEST
        UniversalTimer timerB;
        timerB.setTime();
#endif
        //sort inputs
        int cnt = inputSize;
        memcpy(ids, inputs, sizeof(ui) * cnt);
        if (cnt < 20)
            sort(ids, ids + cnt);
        else threadingSortInputs(cnt);

        nodeCnt = unique(ids, ids + cnt) - ids;
#ifdef TEST
        timerB.resetTimeWithTag("[Construct Graph]-Sort Inputs");
#endif
        //map to [1,nodeCnt)
        nodeCnt++;
        for (int i = 1; i < nodeCnt; ++i) {
            idHash[ids[i - 1]] = i;
        }

#ifdef TEST
        timerB.resetTimeWithTag("[Construct Graph]-Hash");
        printf("%d Nodes in Total\n", nodeCnt - 1);
#endif

        for (int i = 0; i < inputSize; i += 2) {
            ui u = idHash[inputs[i]], v = idHash[inputs[i + 1]];

            inputs[i] = u;
            inputs[i + 1] = v;

            ++inDegrees[v];
            ++outDegrees[u];
        }

#ifdef TEST
        timerB.resetTimeWithTag("[Construct Graph]-Build Graph Remap");
#endif

        int outDegreeCnt = 0;
        int inDegreeCnt = 0;
        for (int i = 1; i < nodeCnt; ++i) {
            G[i] = Node(outDegreeCnt, outDegreeCnt);
            outDegreeCnt += outDegrees[i];
            inDegreeCnt += inDegrees[i];
            inDegreesStart[i+1]=inDegreeCnt;
        }

        for (int i = 0; i < inputSize; i += 2) {
            ui u = inputs[i], v = inputs[i + 1], w = weights[i >> 1];

            edgesG[G[u].ed++] = Edge<ui>(v, w);
        }

#ifdef TEST
        timerB.resetTimeWithTag("[Construct Graph]-Build Graph Add Edge");
        //timerB.printLogs();
#endif
    }

    void sortGraph(int st, int ed) {
        for (int i = st; i < ed; ++i) {
            sort(edgesG + G[i].st, edgesG + G[i].ed);
        }
    }

    inline void threadingSortGraph() {
        if (nodeCnt < 20) sortGraph(1, nodeCnt);
        else {
            int segmentSize = nodeCnt >> 2;
            int st[4] = {1, segmentSize, segmentSize * 2, segmentSize * 3};
            int ed[4] = {segmentSize, segmentSize * 2, segmentSize * 3, nodeCnt};
            auto t1 = thread(&DataLoader::sortGraph,this, st[0], ed[0]);
            auto t2 = thread(&DataLoader::sortGraph,this, st[1], ed[1]);
            auto t3 = thread(&DataLoader::sortGraph,this, st[2], ed[2]);
            auto t4 = thread(&DataLoader::sortGraph,this, st[3], ed[3]);
            t1.join();
            t2.join();
            t3.join();
            t4.join();
        }

        for (int i = 1; i < nodeCnt; ++i)
            edgeCntG += G[i].ed - G[i].st;
    }

    void preprocess(){
        //Data O1
        // 入度为0的点713644，
        // 其中出度为1的629288，出度为2的50429，出度为3的15835,出度为4的5749
        // 若按入度0出度n做拓扑，还可以删去14218个点（713644+14218）
        // 若按入度0出度1做拓扑，还可删去2523个点（629288+2523）
        //Data O2
        // 随机图，SCC=WCC=1
        //Data O3
        // 混合图 WCC=2

        int eliminatedNodeCnt=0;
        for (int i = 1; i < nodeCnt; ++i) {
            if(!tarjan.dfn[i]){
                tarjan.findScc(i);
            }
            if(inDegrees[i]==0 && outDegrees[i]==1){
                scaleFactor[edgesG[G[i].st].to]++;
                ++eliminatedNodeCnt;
            }else scaleFactor[i]+=1;
        }

        if(nodeCnt>100 && tarjan.sccCnt<10 && eliminatedNodeCnt<nodeCnt/10)
            graphType=RANDOM;
    }

    //重建图写的比较一般
    void reconstructGraph(){
        //按照bfs序重排ID，优化内存排布
        //临近点被放在一起
        //对局部稠密图比较有效,对随机图基本无效
        //if(graphType!=SCALE_FREE) return;

        queue<ui> que;
        ui newId=1;
        auto bfs=[&](int st){
            que.push(st);
            revisit[st]=true;

            while(!que.empty()){
                ui u = que.front();
                que.pop();
                //remap u to newId
                old2NewId[u]=newId;
                //remap newId to u
                new2OldId[newId]=u;
                newId++;

                for (int idx = G[u].st; idx < G[u].ed; ++idx) {
                    ui v = edgesG[idx].to;
                    if (!revisit[v]) {
                        revisit[v]=true;
                        que.push(v);
                    }
                }
            }
        };
        for (int i = 1; i < nodeCnt; i++) {
            if (!revisit[i]) bfs(i);
        }

        auto switchArray=[&](ui* dat){
            for (int i = 1; i < nodeCnt; ++i){
                uiTmp[i]=dat[new2OldId[i]];
            }
            memcpy(dat,uiTmp,sizeof(ui)*nodeCnt);
        };

        //根据remappedPos重写ids scaleFactor inDegreesStart edgesG G in/outDegree
        memcpy(edgesGTmp,edgesG,sizeof(Edge<ui>)*edgeCntG);
        memcpy(GTmp,G,sizeof(Node)*nodeCnt);

        switchArray(scaleFactor);
        switchArray(inDegrees);
        switchArray(outDegrees);
        for (int i = 1; i < nodeCnt; ++i){
            uiTmp[i-1]=ids[new2OldId[i]-1];
        }
        memcpy(ids,uiTmp,sizeof(ui)*nodeCnt);
        int inDegreeCnt = 0;
        int curEdgeIdx=0;
        for (int i = 1; i < nodeCnt; ++i){
            inDegreeCnt += inDegrees[i];
            inDegreesStart[i+1]=inDegreeCnt;

            //newId:i oldId:u
            //G[i] ~ st ed
            ui u=new2OldId[i];
            G[i] = Node(curEdgeIdx, curEdgeIdx);
            if(scaleFactor[i]==0) continue;
            for (int idx = GTmp[u].st; idx < GTmp[u].ed; ++idx) {
                edgesG[curEdgeIdx].w=edgesGTmp[idx].w;
                edgesG[curEdgeIdx].to=old2NewId[edgesGTmp[idx].to];
                curEdgeIdx++;
            }
            G[i].ed=curEdgeIdx;
        }
    }
};

template<class T,class E>
struct Solver{
    const T maxValue;
    T distance[NUM_THREAD][MAXN];

    Edge<E> edgesGLocal[MAXE];

    Solver(T maxValue) : maxValue(maxValue) {}

    void solveBlock(int startBid, int threadId) {
        Bundle &bundle = bundles[threadId];
        int startIdx = startBid * blockStride;
        int endIdx = (startBid + 1) * blockStride;
        endIdx = min(nodeCnt, endIdx);
        if (endIdx <= startIdx) return;

        ui *pList = bundle.pList;
        auto *sigma=bundle.sigma;
        ui *stack = bundle.stack;
        decimal *dependency = bundle.dependency;
        decimal *bc = bundle.bc;

        auto *dis = distance[threadId];

        auto* bucketCnt=bundle.bucketCnt;
        auto* bucketElement=bundle.bucketElement;

        for (int i = startIdx; i < endIdx; ++i) {
            if(scaleFactor[i]==0 || outDegrees[i]==0) continue;
            ui head = (ui) i;
#ifdef TEST
            globalTimer.updateProgress(head,nodeCnt-1,head==(ui)nodeCnt-1);
#endif
            ///SSSP Dijkstra
            int stackIdx = 0;
            dis[head] = 0;
            sigma[head].pathCnt = 1;

            ui pqCnt=1;
            bucketElement[0][bucketCnt[0]++]=head;

            for(ui curDis=0;pqCnt;++curDis){
                ui bid=curDis&127;
                if(!bucketCnt[bid]) continue;
                for(ui idx=0;idx<bucketCnt[bid];idx++){
                    ui u=bucketElement[bid][idx];
                    //equal to vis[u]
                    if(dis[u]<curDis) continue;
                    stack[stackIdx++] = u;
                    int lim = G[u].ed;
                    for (int it = G[u].st; it < lim; ++it) {
                        auto v = edgesG[it].to;
                        auto disSum=dis[u] + edgesG[it].w;
                        //无符号型不能用减法
                        //> hit rate 18%  == hit rate 1%
                        if (dis[v] > disSum) {
                            dis[v] = disSum;
                            sigma[v].pathCnt = sigma[u].pathCnt;
                            sigma[v].pListEnd = -((int32_t)u);
                            ui nxtBid=disSum&127;
                            bucketElement[nxtBid][bucketCnt[nxtBid]++]=v;
                            ++pqCnt;
                        } else if (dis[v] == disSum) {
                            //switch from one pred to n
                            int32_t prev=sigma[v].pListEnd;
                            if(prev<0){
                                sigma[v].pListEnd=inDegreesStart[v];
                                pList[sigma[v].pListEnd++]=-prev;
                            }
                            sigma[v].pathCnt += sigma[u].pathCnt;
                            pList[sigma[v].pListEnd++]=u;
                        }
                    }
                }
                pqCnt-=bucketCnt[bid];
                bucketCnt[bid]=0;
            }

            sigma[head].pathCnt = 0;
            dis[head] = maxValue;

            ///Calc BC
            int scale=scaleFactor[head];
            stackIdx--;
            //stack[0] is always head
            for (int si = stackIdx; si > 0; si--) {
                ui v = stack[si];
                //split it to 2 parts
                //dependency[u] += pathCnt[u] * (dependency[v] + 1) / pathCnt[v] = pathCnt[u] *dv;
                //satisfy pathCnt[v]!=0

                int32_t lim=sigma[v].pListEnd;
                int32_t pathCnt=sigma[v].pathCnt;
                const decimal dv=dependency[v] + 1.0 / pathCnt;

                if(lim<0){
                    auto prev=-lim;
                    dependency[prev] += dv;
                }else if(lim>0){
                    for (int idx=inDegreesStart[v];idx<lim;++idx) {
                        //u是v的前继，当更新u时，u还在栈中
                        //栈中所有元素不重复，栈中的点属于DAG
                        //若要判断某条边是否属于DAG，使用dis[v]==dis[u]+edge.w
                        //v更新多个u，u也可能被多个v更新
                        dependency[pList[idx]] += dv;
                    }
                }
//            printf("Head:%d Node:%d Delta:%.3lf\n",head,v,dependency[v]);
                //对于后续涉及的所有结点，更新操作加上scale-1份
                if(scale==1) bc[v] += dependency[v]*pathCnt;
                else bc[v] += dependency[v]*pathCnt*scale;

                sigma[v].pathCnt = 0;
                dependency[v]=0;
                dis[v] = maxValue;
            }
            //没有使用但是需要清零
            dependency[head]=0;
            if(scale>1){
                //对于当前被下放的结点，要加上其前继个数，累加次数为后续结点数
                //may overflow
                bc[head] +=(scale-1)*stackIdx;
            }
        }
    }

    void solveThread(int threadId) {
#ifdef TEST
        UniversalTimer timer;
        timer.setTime();
#endif
        bundles[threadId].threadId = threadId;
        fill(distance[threadId],distance[threadId]+nodeCnt,maxValue);

        while (currentBlock < blockNum) {
            solveBlock(atomic_fetch_add_explicit(&currentBlock, 1, std::memory_order_relaxed), threadId);
        }

#ifdef TEST
        //timer.logTime("[Thread " + to_string(threadId) + "].");
#endif
    }

    void solveWithThreads() {
        for(int i=0;i<edgeCntG;++i){
            edgesGLocal[i].to=edgesG[i].to;
            edgesGLocal[i].w=edgesG[i].w;
        }
        if (nodeCnt < 20) {
            blockNum = 1;
            blockStride = nodeCnt;
            solveThread(0);
            memcpy(bcSum, bundles[0].bc, sizeof(decimal) * nodeCnt);
        } else {
            blockNum = min(NUM_BLOCK, nodeCnt);
            blockStride = (nodeCnt + blockNum - 1) / blockNum;
            std::thread threads[NUM_THREAD];
            for (int i = 0; i < NUM_THREAD; ++i) {
                threads[i] = thread(&Solver::solveThread,this, i);
            }
            for (auto &thread : threads) {
                thread.join();
            }
            for (int i = 0; i < NUM_THREAD; ++i) {
                auto &bc = bundles[i].bc;
                for (int idx = 0; idx < nodeCnt; ++idx)
                    bcSum[idx] += bc[idx];
            }
        }
    }

    void prepareResults(){
        for (int i = 0; i < nodeCnt; i++) {
            sortedBCIdx[i] = i;
        }
        auto cmp=[&](int l, int r) {
            //做完重映射后，ids里面的ID不再有序
            if(fabs(bcSum[l] - bcSum[r])<1e-4)
                return ids[l-1]<ids[r-1];
            return bcSum[l] > bcSum[r];
        };
        sort(sortedBCIdx + 1, sortedBCIdx + nodeCnt, cmp);
    }

    void save_fwrite(const string &outputFile) {
        prepareResults();

        FILE *fp = fopen(outputFile.c_str(), "wb");
        char buf[64];
        for (int i = 1; i < min(nodeCnt, 101); i++) {
            //LF for long double
            int idx = sprintf(buf, "%d,%.3lf\n", ids[sortedBCIdx[i] - 1], bcSum[sortedBCIdx[i]]);
            fwrite(buf, idx, sizeof(char), fp);
        }
        fclose(fp);
    }
};

////主要类声明
DataLoader dataLoader;
//<dis,w>
//标准型：路径ull，权重ui
Solver<ull,ui> solver64(UINT64_MAX);
//路径压缩：路径ui，权重ui
Solver<ui,ui> solver32(UINT32_MAX);
//路径压缩：路径us，权重us
Solver<us,us> solver16(UINT16_MAX);
////

int main(int argc, char *argv[]) {
    //vectorTest();

#ifdef TEST
    UniversalTimer timerA, timerB;
#endif

#ifdef CONFIG_USE_MMAP
    //调低进程的友善值
    nice(-20);
#endif

    string testFile;
    if (argc > 1)
        testFile = string(argv[1]);
    else {
#ifdef TEST
//          testFile = "test_data_SFN.N1560268.E200W.A18875018.txt";
//        testFile = "test_data.final.txt";
//        testFile = "test_iot/data3.txt";
//        testFile = "test_data.final.toy4.txt";
//        testFile = "test_data.final.toy.txt";
//        testFile = "test_data_final1_N1600443_E250W.txt";
//        testFile = "test_data.N111314.E200W.A19630345.txt";

//        testFile = "test_data.O1.final.txt";
//        testFile = "test_data.O2.final.txt";
        testFile = "test_data.O3.final.txt";
#else
        testFile = "/data/test_data.txt";
#endif
    }

    string outputFile = "/projects/student/result.txt";
#ifdef TEST
    outputFile = "output.txt";
    timerA.setTime();
    timerB.setTime();
#endif

    dataLoader.parseInput(testFile);
#ifdef TEST
    timerB.resetTimeWithTag("Read Input File");
#endif

    dataLoader.constructGraph();
#ifdef TEST
    timerB.resetTimeWithTag("Construct Graph");
#endif

    dataLoader.threadingSortGraph();
#ifdef TEST
    printf("[G]-%d Edges in Total After Preprocess\n", edgeCntG);
    timerB.resetTimeWithTag("Sort Graph");
#endif

    dataLoader.preprocess();
#ifdef TEST
    printf("%d SCC found. GraphType: [%s]\n",dataLoader.tarjan.sccCnt,graphType==RANDOM? "Random":"Scale Free");
    printf("Fit W-UINT32: %d W-UINT16：%d WeightOOR-%d\n",weightFitUI32,weightFitUI16,weightOutOfRange);
    timerB.resetTimeWithTag("Find SCC");
    globalTimer.setTime();
#endif
    dataLoader.reconstructGraph();
#ifdef TEST
    timerB.resetTimeWithTag("Reconstruct Graph");
#endif

    if(weightFitUI16) {
        solver16.solveWithThreads();
        solver16.save_fwrite(outputFile);
    }else
        if(weightFitUI32){
        solver32.solveWithThreads();
        solver32.save_fwrite(outputFile);
    }else{
        solver64.solveWithThreads();
        solver64.save_fwrite(outputFile);
    }
#ifdef TEST
    timerB.resetTimeWithTag("Solve & Output");
#endif

#ifdef CF
    usleep(timerA.getElapsedTimeMS()*4*1000);
#endif

#ifdef TEST
    timerB.printLogs();
    timerA.logTime("Whole Process");
#endif

    _exit(0);
    return 0;
}
