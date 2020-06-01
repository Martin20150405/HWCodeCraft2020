#include <bits/stdc++.h>

using namespace std;

///@Author 小白兔奶糖
///@Date 2020.5
///@Algorithm +4-3

#include <ext/pb_ds/hash_policy.hpp>
#include <ext/pb_ds/assoc_container.hpp>

//#define CONFIG_USE_NEON
#define CONFIG_USE_MMAP
#define TEST

#ifdef CONFIG_USE_NEON

#include <arm_neon.h>

#endif

#ifdef CONFIG_USE_MMAP

#include <sys/mman.h>
#include <sys/stat.h>

#endif

#include <fcntl.h>
#include <unistd.h>

///Timer
#ifdef TEST
#ifdef _WIN32   // windows

#include <sysinfoapi.h>

#else   // unix

#include <sys/time.h>

#endif

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
        printf("Time consumed for %-20s is %d ms.\n", tag.c_str(), elapsedTimeMS);
    }

    void logTime(string tag) {
        logTimeImpl(tag, getElapsedTimeMS());
    }

    void resetTimeWithTag(string tag) {
        logPairs.emplace_back(make_pair(tag, getElapsedTimeMS()));
        setTime();
    }

    void printLogs() {
        for (auto x:logPairs)
            logTimeImpl(x.first, x.second);
    }
};

#endif
///


#ifdef CONFIG_USE_NEON

//128bit 16B one op
//will copy 16 if [1,16)
inline void memcpy16B(void *dest, void *src, size_t count) {
    int i;
    unsigned long *s = (unsigned long *) src;
    unsigned long *d = (unsigned long *) dest;
    count /= 16;
    for (i = 0; i <= count; i++) {
        vst1q_u64(&d[0], vld1q_u64(&s[0]));
        d += 2;
        s += 2;
    }
}

inline void memcpy16BX1(void *d, void *s) {
    vst1q_u64((unsigned long *) d, vld1q_u64((unsigned long *) s));
}

inline void memcpy32BX1(void *d, void *s) {
    vst1q_u64((unsigned long *) d, vld1q_u64((unsigned long *) s));
    vst1q_u64(((unsigned long *) d)+2, vld1q_u64(((unsigned long *) s)+2));
}

#endif

typedef unsigned long long ull;
typedef unsigned int ui;

//MAXN won't exceed MAXE if only consider u
const int MAXE = 2e6 + 8;
const int MAXN = MAXE;
const int MAX_LOOP = 2e7 + 8;
const int MAX_DEPTH = 7;
const int MIN_DEPTH = 3;
const int DEPTH_NUM = MAX_DEPTH - MIN_DEPTH + 1;
const int MAX_TOTAL_PRE3_PATH = MAXN;
const unsigned int MAX_MEMORY_POOL_SIZE = 3u * 1024 * 1024 * 1024; //3G

#define NUM_THREAD 4
#define NUM_BLOCK 8192

//~96M
#define SIZE_PER_ANS_BUFFER  96*1048576


//线上ID不连续
int inputs[MAXE * 3]; //u-v-w pairs
__gnu_pbds::gp_hash_table<int, int> idHash; //remap sorted id to 1...n
//tsl::robin_map<int, int> idHash; //remap sorted id to 1...n
int tmpIds[MAXN]; //sorted u
int ids[MAXN]; //sorted u

struct Edge {
    int to, w;

    Edge() {}

    Edge(int to, int w) : to(to), w(w) {}

    bool operator<(const Edge &rhs) const {
        return to < rhs.to;
    }
};

Edge edgesG[MAXE];
Edge edgesGInv[MAXE];

struct Node {
    //record G[st,ed) for Node i
    int st, ed;

    Node() {}

    Node(int st, int ed) : st(st), ed(ed) {}
};

//默认情况下，起止是0
Node G[MAXN]; //u:v-w
Node GInv[MAXN]; //v:u-w

char idsComma[MAXN][16]; //最大长度为11，长度10+逗号/换行符 ，长度存储在第0位

int inDegrees[MAXN];
int outDegrees[MAXN];
int que[MAXN];

#define mp(a, b) ((ull)a<<32 | b)

struct P3Info {
    //->pre3[w3]->pre2[w2]->pre1[w1]->head
    ull pre21;
    int pre2, pre1, w1, w3;

    P3Info() {}

    P3Info(int pre2, int pre1, int w1, int w3) : pre2(pre2), pre1(pre1), w1(w1), w3(w3) {
        pre21 = mp(pre2, pre1);
    }

    bool operator<(const P3Info &rhs) const {
        return pre21 < rhs.pre21;
    }
};

struct MemoryPool {
    char *ptr;
    mutex ptrLock;

    MemoryPool() {}

    MemoryPool(size_t sizeInBytes) {
        init(sizeInBytes);
    }

    inline void init(size_t sizeInBytes) {
        ptr = (char *) malloc(sizeInBytes);
    }

    inline char *allocateCharBuffer(int sz) {
        ptrLock.lock();
        ptr += sz;
        ptrLock.unlock();
        return ptr;
    }
};

MemoryPool ansMemoryPool(MAX_MEMORY_POOL_SIZE);

struct Bundle {
    //int threadId;

    ////答案存储
    //thread*depthRange*AnsNum
    int ansCnt; //sum of [3,4,5,6,7][threadId]
    char *bufferPtr[DEPTH_NUM];
    int bufferSize[DEPTH_NUM];


    ////中间变量
    Node GLocal[MAXN];
    Node GInvLocal[MAXN];

    bool vis[MAXN];

    //->pre3[w3]->pre2[w2]->pre1[w1]->head
    //reachableP3[0][pre3]=head indicates pre3->pre2->pre1->head
    //reachableP3[1][pre3]=addr[pre3] indicates addr of path list
    //reachableP3[2][pre3]=ed indicates end of path list
    int reachableP3[3][MAXN];

//    bool reachableP3[MAXN];

    P3Info *pre12List[MAXN];

    //<pre3,pre2,pre1> should be sorted in ascending order
    //DIDIM2:pre12 List for every P3
    //数组模拟链表（链式前向星）存储
    P3Info pre12Buffer[MAX_TOTAL_PRE3_PATH];
    int pre12Head[MAXN]; //store idx of head in buffer
    int pre12Nxt[MAX_TOTAL_PRE3_PATH]; //store next idx of current idx
    int pre3Set[MAXN]; //store all head
    ////

    inline bool checkNode(int i) {
        return GLocal[i].st < GLocal[i].ed && GInvLocal[i].st < GInvLocal[i].ed;
    }
};

Bundle bundles[NUM_THREAD];

struct Block {
    //solved by thread i using bundles[i]'s buffer
    int bufferCnt[DEPTH_NUM];
    char *startAddr[DEPTH_NUM][8];
    char *endAddr[DEPTH_NUM][8];
};

Block blocks[NUM_BLOCK];
int blockNum;
int blockStride;
atomic_int currentBlock(0);
////

int nodeCnt = 0;
int inputSize = 0;

void parseInput(string &testFile) {
#ifndef CONFIG_USE_MMAP
    FILE *file = fopen(testFile.c_str(), "r");
    int u, v, c;
    while (fscanf(file, "%d,%d,%d", &u, &v, &c) != EOF) {
        inputs[inputSize++] = u;
        inputs[inputSize++] = v;
        inputs[inputSize++] = c;
    }
#else
    int fd = open(testFile.c_str(), O_RDONLY);
    int fileLength = lseek(fd, 0, SEEK_END);
    char *buffer =
            (char *) mmap(NULL, fileLength, PROT_READ, MAP_PRIVATE, fd, 0);

    //as buffer reaching bufferEnd
    //value won't be in ['0','9']
    while (*buffer>='0') {
        int u=0,v=0,w=0;
        while (*buffer >='0') {
            u = u * 10 + *(buffer++) - '0';
        }
        ++buffer;
        while (*buffer >='0') {
            v = v * 10 + *(buffer++) - '0';
        }
        ++buffer;
        while (*buffer >='0') {
            w = w * 10 + *(buffer++) - '0';
        }
        if(*buffer=='\r') ++buffer;
        ++buffer;
        inputs[inputSize++] = u;
        inputs[inputSize++] = v;
        inputs[inputSize++] = w;
    }
#endif
}


inline char *int2charX1(int x, char *ptr) {
    do {
        *(--ptr) = x % 10 + '0';
        x /= 10;
    } while (x);
    return ptr;
}

static const char digitsTable2[] = {
        "00010203040506070809"
        "10111213141516171819"
        "20212223242526272829"
        "30313233343536373839"
        "40414243444546474849"
        "50515253545556575859"
        "60616263646566676869"
        "70717273747576777879"
        "80818283848586878889"
        "90919293949596979899",
};

inline char *int2charX2(int x, char *ptr) {
    int idx;
    while (x >= 100) {
        idx = (x % 100) << 1;
        x /= 100;
        *(--ptr) = digitsTable2[idx + 1];
        *(--ptr) = digitsTable2[idx];
    }
    if (x < 10) *(--ptr) = '0' + x;
    else {
        idx = x << 1;
        *(--ptr) = digitsTable2[idx + 1];
        *(--ptr) = digitsTable2[idx];
    }
    return ptr;
}

inline void sortInputs(int st,int ed){
    sort(ids+st, ids+ed);
}

void merge2Result(int st1,int ed1,int st2,int ed2,int* src,int dstSt,int* dst){
    //merge 2 sorted array
    int idx=0,lim=(ed1-st1)+(ed2-st2);
    while(idx<lim){
        int mins=INT_MAX;
        bool flag=false;
        if(st1<ed1 && src[st1]<mins){
            mins=src[st1];
        }
        if(st2<ed2 && src[st2]<mins){
            flag=true;
            mins=src[st2];
        }
        dst[dstSt++]=mins;
        if(flag) ++st2;
        else ++st1;
        ++idx;
    }
}

inline void threadingSortInputs(int cnt){
    //四路排序
    int segmentSize = cnt >> 2;
    int st[4]={0,segmentSize,segmentSize*2,segmentSize*3};
    int ed[4]={segmentSize,segmentSize*2,segmentSize*3,cnt};

    auto t1 = thread(sortInputs, st[0], ed[0]);
    auto t2 = thread(sortInputs, st[1], ed[1]);
    auto t3 = thread(sortInputs, st[2], ed[2]);
    auto t4 = thread(sortInputs, st[3], ed[3]);
    t1.join();
    t2.join();
    t3.join();
    t4.join();

    //4归并至2
    auto t5 = thread(merge2Result,st[0], ed[0], st[1], ed[1], ids, st[0], tmpIds);
    auto t6 = thread(merge2Result,st[2], ed[2], st[3], ed[3], ids, st[2], tmpIds);
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
    //update nodeCnt idsComma idsLF ids idHash G
    //update inDegrees outDegrees
    int cnt = 0;
    for (int i = 0; i < inputSize; i += 3) {
        ids[cnt++] = inputs[i];
    }
    if(cnt<20)
        sort(ids, ids + cnt);
    else threadingSortInputs(cnt);

    nodeCnt = unique(ids, ids + cnt) - ids;
#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Sort Inputs");
#endif
    //map to [1,nodeCnt)
    nodeCnt++;
    //idHash.reserve(nodeCnt);
    char buf[12];
    char *ptrEd = buf + 11;
    int tmp;
    for (int i = 1; i < nodeCnt; ++i) {
        idHash[tmp = ids[i - 1]] = i;
        char *ptr = int2charX2(tmp, ptrEd);
        *ptrEd = ',';
        char len = ptrEd - ptr + 1;
        memcpy(idsComma[i], ptr, len);
        idsComma[i][15] = len;
    }

#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Hash");
    printf("%d Nodes in Total\n", nodeCnt - 1);
#endif

    for (int i = 0; i < inputSize; i += 3) {
        auto it=idHash.find(inputs[i + 1]);
        if (it == idHash.end() || inputs[i + 2] == 0) {
            inputs[i] = -1;
            continue;
        }
        int u = idHash[inputs[i]], v = it->second;

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
        G[i]=Node(outDegreeCnt,outDegreeCnt);
        outDegreeCnt+=outDegrees[i];
        GInv[i]=Node(inDegreeCnt,inDegreeCnt);
        inDegreeCnt+=inDegrees[i];
    }

    for (int i = 0; i < inputSize; i += 3) {
        if (inputs[i] < 0) continue;
        int u = inputs[i], v = inputs[i + 1], w = inputs[i + 2];

        edgesG[G[u].ed++]=Edge(v, w);
        edgesGInv[GInv[v].ed++]=Edge(u, w);
    }

#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Build Graph Add Edge");
    timerB.printLogs();
#endif
}

void topoSort() {
    //remove nodes with inDegree==0
    int queueIdx = -1;
    for (int i = 1; i < nodeCnt; i++) {
        if (0 == inDegrees[i])
            que[++queueIdx] = i;
    }
    while (queueIdx >= 0) {
        int u = que[queueIdx--];

        for (int idx = G[u].st; idx<G[u].ed; ++idx) {
            int v = edgesG[idx].to;
            --outDegrees[u];
            if (0 == --inDegrees[v]) {
                que[++queueIdx] = v;
            }
        }
    }

    //remove nodes with outDegree==0
    for (int i = 1; i < nodeCnt; i++) {
        if (0 == outDegrees[i])
            que[++queueIdx] = i;
    }

    while (queueIdx >= 0) {
        int v = que[queueIdx--];
        for (int idx = GInv[v].st; idx<GInv[v].ed; ++idx) {
            int u = edgesGInv[idx].to;
            --inDegrees[v];
            if (0 == --outDegrees[u]) {
                que[++queueIdx] = u;
            }
        }
    }
}

void reconstructGraph(int st,int ed) {
    for (int i = st; i < ed; ++i) {
        if (inDegrees[i]==0 || outDegrees[i] == 0) {
            G[i].ed = G[i].st;
            GInv[i].ed=GInv[i].st;
        } else {
            int idx = G[i].st;
            for (int j = idx; j < G[i].ed; ++j) {
                int v = edgesG[j].to;
                if (inDegrees[v] && outDegrees[v]) edgesG[idx++] = edgesG[j];
            }
            G[i].ed = idx;
            sort(edgesG + G[i].st, edgesG + G[i].ed);

            idx=GInv[i].st;
            for(int j=idx;j<GInv[i].ed;++j){
                int u=edgesGInv[j].to;
                if(inDegrees[u] && outDegrees[u]) edgesGInv[idx++]=edgesGInv[j];
            }
            GInv[i].ed=idx;
            sort(edgesGInv+GInv[i].st,edgesGInv+GInv[i].ed);
        }
    }
}

inline void threadingReconstructGraph() {
    if (nodeCnt < 20) reconstructGraph(1, nodeCnt);
    else {
        int segmentSize = nodeCnt >> 2;
        int st[4]={1,segmentSize,segmentSize*2,segmentSize*3};
        int ed[4]={segmentSize,segmentSize*2,segmentSize*3,nodeCnt};
        auto t1 = thread(reconstructGraph, st[0], ed[0]);
        auto t2 = thread(reconstructGraph, st[1], ed[1]);
        auto t3 = thread(reconstructGraph, st[2], ed[2]);
        auto t4 = thread(reconstructGraph, st[3], ed[3]);
        t1.join();
        t2.join();
        t3.join();
        t4.join();
    }
}

inline bool checkWeight(ull x, ull y) {
    //y/x in [0.2,3]
    //return x <= 5ll * y && y <= 3ll * x;
    return (y << 2) + y >= x && (x << 1) + x >= y;
}

int constructP3(int head, Bundle &bundle, Block &block) {
    int pre1 = 0, pre2 = 0, pre3 = 0, w1 = 0, w2 = 0, w3 = 0, p3Cnt = 0, pathCnt = 0;
    //->pre3[w3]->pre2[w2]->pre1[w1]->head
    //head<min(pre1,pre2,pre3)
    //pre2!=pre1 pre3!=pre1 > head is assured here
    //pre2!=pre3 pre1!=pre3 > head and pre2,pre1 not visited is assured in dfs

    auto &pre12Buffer = bundle.pre12Buffer;
    auto &pre12Head = bundle.pre12Head;
    auto &pre12Nxt = bundle.pre12Nxt;
    auto &pre3Set = bundle.pre3Set;

    auto &pre12List = bundle.pre12List;

    auto &GInvLocal = bundle.GInvLocal;
    auto &reachableP3 = bundle.reachableP3;
    auto &vis = bundle.vis;

    int &st1 = GInvLocal[head].st;
    int lim1 = GInvLocal[head].ed;

    while (st1 < lim1 && edgesGInv[st1].to < head) ++st1;

    for (int it1 = st1; it1 < lim1; ++it1) {
        pre1 = edgesGInv[it1].to;
        if (pre1 == head || vis[pre1])
            continue;
        w1 = edgesGInv[it1].w;
//        printf("->%d[%d]->%d\n",ids[pre1-1],w1,ids[head-1]);
        int &st2 = GInvLocal[pre1].st;
        int lim2 = GInvLocal[pre1].ed;
        while (st2 < lim2 && edgesGInv[st2].to < head) ++st2;
        for (int it2 = st2; it2 < lim2; ++it2) {
            pre2 = edgesGInv[it2].to;
            w2 = edgesGInv[it2].w;
            if (pre2 == head || vis[pre2] || pre2 == pre1 || !checkWeight(w2, w1)) //21
                continue;
//            printf("->%d[%d]->%d[%d]->%d\n",ids[pre2-1],w2,ids[pre1-1],w1,ids[head-1]);
            int &st3 = GInvLocal[pre2].st;
            int lim3 = GInvLocal[pre2].ed;
            while (st3 < lim3 && edgesGInv[st3].to < head) ++st3;
            for (int it3 = st3; it3 < lim3; ++it3) {
                pre3 = edgesGInv[it3].to;
                w3 = edgesGInv[it3].w;
                if (pre3 == head) {
                    if (!checkWeight(w1, w3)) continue;
                } else if (vis[pre3] || pre3 == pre1) continue;
                if (!checkWeight(w3, w2)) continue;  //32

//                printf("->%d[%d]->%d[%d]->%d[%d]->%d\n",ids[pre3-1],w3,ids[pre2-1],w2,ids[pre1-1],w1,ids[head-1]);
                if (reachableP3[0][pre3] != head) {
                    reachableP3[0][pre3] = head;
                    pre12Head[pre3] = -1;
                    pre3Set[p3Cnt++] = pre3;
                }
                pre12Buffer[pathCnt] = P3Info(pre2, pre1, w1, w3);
                pre12Nxt[pathCnt] = pre12Head[pre3];
                pre12Head[pre3] = pathCnt++;
            }
        }
    }
    if (p3Cnt == 0) return 0;
    int lastPos = 0;
    for (int i = 0; i < p3Cnt; ++i) {
        pre3 = pre3Set[i];
        int curPos = lastPos;
        for (int idx = pre12Head[pre3]; idx != -1; idx = pre12Nxt[idx]) {
            pre12List[curPos++] = &pre12Buffer[idx];
        }
        if (curPos - lastPos > 1)
            sort(pre12List + lastPos, pre12List + curPos,
                 [&](const P3Info *lhs, const P3Info *rhs) -> bool { return *lhs < *rhs; });
        reachableP3[1][pre3] = lastPos;
        reachableP3[2][pre3] = curPos;
        lastPos = curPos;
    }

    return p3Cnt;
}

//check [3] in construct P3
//check [4] in depth [1]
//check [5] in depth [2]
//check [6] in depth [3]
//check [7] in depth [4]
void dfsP3NR(int startIdx, int endIdx, Bundle &bundle, Block &block) {
    auto &vis = bundle.vis;
    auto &reachableP3 = bundle.reachableP3;
    auto &pre12List = bundle.pre12List;
    auto &GLocal = bundle.GLocal;
    int ansCnt = 0;

    auto checkBuffer = [&](int curDepthMinus3) {
        if (bundle.bufferSize[curDepthMinus3] < 80) {
            auto &bufferSize = bundle.bufferSize[curDepthMinus3];
            auto &bufferPtr = bundle.bufferPtr[curDepthMinus3];
            block.endAddr[curDepthMinus3][block.bufferCnt[curDepthMinus3]++] = bufferPtr;
            bufferSize = SIZE_PER_ANS_BUFFER;
            bufferPtr = ansMemoryPool.allocateCharBuffer(bufferSize);
            block.startAddr[curDepthMinus3][block.bufferCnt[curDepthMinus3]] = bufferPtr;
#ifdef TEST
            //printf("[Thread %d]: Allocating %dMB in depth %d\n", bundle.threadId, bufferSize / 1024 / 1024, depth);
#endif
        }
    };

    for (int head = startIdx; head < endIdx; ++head) {
        //no p1 skip
        if (!bundle.checkNode(head) || constructP3(head, bundle, block) == 0) continue;
        //head/f1[w12]->f2[w23]->f3[w34]->f4[w45]->p3[w3]->p2[w2]->p1[w1]->head
        //<w12,w23> <w23,w34> <w34,w45> 在循环中检查
        //<lst,w3> and <w1,w12> 在记录路径时检查
        //<w3,w2> <w2,w1> 在构建P3与P2时检查
        vis[head] = true;

        int curPos, pathAddrEd, pre2, pre1;

        //check loop 3
        if (reachableP3[0][head] == head) {
            curPos = reachableP3[1][head];
            pathAddrEd = reachableP3[2][head];
            for (; curPos < pathAddrEd; ++curPos) {
                //<pre2,pre1>,<w1,w3>
                auto &pathInfo = *pre12List[curPos];

                //dump path
                checkBuffer(0);
                auto &bufferPtr = bundle.bufferPtr[0];
                auto &bufferSize = bundle.bufferSize[0];
                auto originBufferPtr = bufferPtr;

                pre2 = pathInfo.pre2;
                pre1 = pathInfo.pre1;

                memcpy(bufferPtr, idsComma[head], 16);
                bufferPtr += idsComma[head][15];
                memcpy(bufferPtr, idsComma[pre2], 16);
                bufferPtr += idsComma[pre2][15];
                memcpy(bufferPtr, idsComma[pre1], 16);
                bufferPtr += idsComma[pre1][15];

                *(bufferPtr - 1) = '\n';
                bufferSize -= (bufferPtr - originBufferPtr);

                ++ansCnt;
            }
        }

        int &st1 = GLocal[head].st;
        int lim1 = GLocal[head].ed;
        while (st1 < lim1 && edgesG[st1].to <= head) ++st1;
        for (int it1 = st1; it1 < lim1; ++it1) {
            int &f2 = edgesG[it1].to;
            if (vis[f2]) continue;
            int &w12 = edgesG[it1].w;
            vis[f2] = true;
            //check loop 4
            if (reachableP3[0][f2] == head) {
                curPos = reachableP3[1][f2];
                pathAddrEd = reachableP3[2][f2];
                for (; curPos < pathAddrEd; ++curPos) {
                    //<pre2,pre1>,<w1,w3>
                    auto &pathInfo = *pre12List[curPos];
                    pre2 = pathInfo.pre2;
                    pre1 = pathInfo.pre1;
                    //vis & weight check
                    if (vis[pre1] || vis[pre2] ||
                        !checkWeight(w12, pathInfo.w3) ||
                        !checkWeight(pathInfo.w1, w12))
                        continue;
                    //dump path
                    checkBuffer(1);
                    auto &bufferPtr = bundle.bufferPtr[1];
                    auto &bufferSize = bundle.bufferSize[1];
                    auto originBufferPtr = bufferPtr;

                    memcpy(bufferPtr, idsComma[head], 16);
                    bufferPtr += idsComma[head][15];
                    memcpy(bufferPtr, idsComma[f2], 16);
                    bufferPtr += idsComma[f2][15];
                    memcpy(bufferPtr, idsComma[pre2], 16);
                    bufferPtr += idsComma[pre2][15];
                    memcpy(bufferPtr, idsComma[pre1], 16);
                    bufferPtr += idsComma[pre1][15];

                    *(bufferPtr - 1) = '\n';
                    bufferSize -= (bufferPtr - originBufferPtr);

                    ++ansCnt;
                }
            }
            int &st2 = GLocal[f2].st;
            int lim2 = GLocal[f2].ed;
            while (st2 < lim2 && edgesG[st2].to <= head) ++st2;
            for (int it2 = st2; it2 < lim2; ++it2) {
                int &f3 = edgesG[it2].to;
                int &w23 = edgesG[it2].w;
                if (!checkWeight(w12, w23) || vis[f3]) continue;
                vis[f3] = true;
                //check loop 5
                if (reachableP3[0][f3] == head) {
                    curPos = reachableP3[1][f3];
                    pathAddrEd = reachableP3[2][f3];
                    for (; curPos < pathAddrEd; ++curPos) {
                        //<pre2,pre1>,<w1,w3>
                        auto &pathInfo = *pre12List[curPos];
                        pre2 = pathInfo.pre2;
                        pre1 = pathInfo.pre1;
                        //vis & weight check
                        if (vis[pre1] || vis[pre2] ||
                            !checkWeight(w23, pathInfo.w3) ||
                            !checkWeight(pathInfo.w1, w12))
                            continue;
                        //dump path
                        checkBuffer(2);

                        auto &bufferPtr = bundle.bufferPtr[2];
                        auto &bufferSize = bundle.bufferSize[2];
                        auto originBufferPtr = bufferPtr;

                        memcpy(bufferPtr, idsComma[head], 16);
                        bufferPtr += idsComma[head][15];
                        memcpy(bufferPtr, idsComma[f2], 16);
                        bufferPtr += idsComma[f2][15];
                        memcpy(bufferPtr, idsComma[f3], 16);
                        bufferPtr += idsComma[f3][15];
                        memcpy(bufferPtr, idsComma[pre2], 16);
                        bufferPtr += idsComma[pre2][15];
                        memcpy(bufferPtr, idsComma[pre1], 16);
                        bufferPtr += idsComma[pre1][15];

                        *(bufferPtr - 1) = '\n';
                        bufferSize -= (bufferPtr - originBufferPtr);

                        ++ansCnt;

                    }
                }
                int &st3 = GLocal[f3].st;
                int lim3 = GLocal[f3].ed;
                while (st3 < lim3 && edgesG[st3].to <= head) ++st3;
                for (int it3 = st3; it3 < lim3; ++it3) {
                    int &f4 = edgesG[it3].to;
                    int &w34 = edgesG[it3].w;
                    if (!checkWeight(w23, w34) || vis[f4]) continue;
                    vis[f4] = true;
                    //check loop 6
                    if (reachableP3[0][f4] == head) {
                        curPos = reachableP3[1][f4];
                        pathAddrEd = reachableP3[2][f4];
                        for (; curPos < pathAddrEd; ++curPos) {
                            //<pre2,pre1>,<w1,w3>
                            auto &pathInfo = *pre12List[curPos];
                            pre2 = pathInfo.pre2;
                            pre1 = pathInfo.pre1;
                            //vis & weight check
                            if (vis[pre1] || vis[pre2] ||
                                !checkWeight(w34, pathInfo.w3) ||
                                !checkWeight(pathInfo.w1, w12))
                                continue;
                            //dump path
                            checkBuffer(3);

                            auto &bufferPtr = bundle.bufferPtr[3];
                            auto &bufferSize = bundle.bufferSize[3];
                            auto originBufferPtr = bufferPtr;

                            memcpy(bufferPtr, idsComma[head], 16);
                            bufferPtr += idsComma[head][15];
                            memcpy(bufferPtr, idsComma[f2], 16);
                            bufferPtr += idsComma[f2][15];
                            memcpy(bufferPtr, idsComma[f3], 16);
                            bufferPtr += idsComma[f3][15];
                            memcpy(bufferPtr, idsComma[f4], 16);
                            bufferPtr += idsComma[f4][15];
                            memcpy(bufferPtr, idsComma[pre2], 16);
                            bufferPtr += idsComma[pre2][15];
                            memcpy(bufferPtr, idsComma[pre1], 16);
                            bufferPtr += idsComma[pre1][15];

                            *(bufferPtr - 1) = '\n';
                            bufferSize -= (bufferPtr - originBufferPtr);

                            ++ansCnt;
                        }
                    }
                    int &st4 = GLocal[f4].st;
                    int lim4 = GLocal[f4].ed;
                    while (st4 < lim4 && edgesG[st4].to <= head) ++st4;
                    for (int it4 = st4; it4 < lim4; ++it4) {
                        int &f5 = edgesG[it4].to;
                        int &w45 = edgesG[it4].w;
                        if (reachableP3[0][f5] != head || !checkWeight(w34, w45) || vis[f5]) continue;
                        vis[f5] = true;
                        //check loop 7
                        curPos = reachableP3[1][f5];
                        pathAddrEd = reachableP3[2][f5];
                        for (; curPos < pathAddrEd; ++curPos) {
                            //<pre2,pre1>,<w1,w3>
                            auto &pathInfo = *pre12List[curPos];
                            pre2 = pathInfo.pre2;
                            pre1 = pathInfo.pre1;
                            //vis & weight check
                            if (vis[pre1] || vis[pre2] ||
                                !checkWeight(w45, pathInfo.w3) ||
                                !checkWeight(pathInfo.w1, w12))
                                continue;
                            //dump path
                            checkBuffer(4);

                            auto &bufferPtr = bundle.bufferPtr[4];
                            auto &bufferSize = bundle.bufferSize[4];
                            auto originBufferPtr = bufferPtr;

                            memcpy(bufferPtr, idsComma[head], 16);
                            bufferPtr += idsComma[head][15];
                            memcpy(bufferPtr, idsComma[f2], 16);
                            bufferPtr += idsComma[f2][15];
                            memcpy(bufferPtr, idsComma[f3], 16);
                            bufferPtr += idsComma[f3][15];
                            memcpy(bufferPtr, idsComma[f4], 16);
                            bufferPtr += idsComma[f4][15];
                            memcpy(bufferPtr, idsComma[f5], 16);
                            bufferPtr += idsComma[f5][15];
                            memcpy(bufferPtr, idsComma[pre2], 16);
                            bufferPtr += idsComma[pre2][15];
                            memcpy(bufferPtr, idsComma[pre1], 16);
                            bufferPtr += idsComma[pre1][15];

                            *(bufferPtr - 1) = '\n';
                            bufferSize -= (bufferPtr - originBufferPtr);

                            ++ansCnt;
                        }
                        vis[f5] = false;
                    }
                    vis[f4] = false;
                }
                vis[f3] = false;
            }
            vis[f2] = false;
        }
    }
    bundle.ansCnt += ansCnt;
}

void solveBlock(int startBid, int threadId) {
    Bundle &bundle = bundles[threadId];
    int startIdx = startBid * blockStride;
    int endIdx = (startBid + 1) * blockStride;
    endIdx = min(nodeCnt, endIdx);
    if (endIdx <= startIdx) return;

#ifdef TEST
//   printf("[Thread %d] is solving Block [%d,%d):[%d,%d)\n", threadId, startBid,endBid, startIdx, endIdx);
//    UniversalTimer timer;
//    timer.setTime();
#endif
    //init block
    Block &block = blocks[startBid];
    for (int i = 0; i < DEPTH_NUM; ++i) {
        block.startAddr[i][block.bufferCnt[i]] = bundle.bufferPtr[i];
    }

    dfsP3NR(startIdx, endIdx, bundle, block);

    for (int i = 0; i < DEPTH_NUM; ++i) {
        block.endAddr[i][block.bufferCnt[i]++] = bundle.bufferPtr[i];
    }

#ifdef TEST
    //    timer.logTime("Current Block");
#endif

}

void solveThread(int threadId) {
#ifdef TEST
    UniversalTimer timer;
    timer.setTime();
#endif
    //bundles[threadId].threadId = threadId;
    auto &GLocal = bundles[threadId].GLocal;
    auto &vis = bundles[threadId].vis;
    memcpy(GLocal, G, sizeof(Node) * nodeCnt);
    memcpy(bundles[threadId].GInvLocal, GInv, sizeof(Node) * nodeCnt);
    for (int i = 0; i < nodeCnt; ++i) {
        if (G[i].ed <= G[i].st) vis[i] = true;
    }
    while (currentBlock < blockNum) {
        solveBlock(currentBlock++, threadId);
    }

#ifdef TEST
    timer.logTime("[Thread " + to_string(threadId) + "].");
#endif
}

void solveWithThreads() {
    currentBlock = 0;
    if (nodeCnt < 20) {
        blockNum = 1;
        blockStride = nodeCnt;
        solveThread(0);
    } else {
        blockNum = min(NUM_BLOCK, nodeCnt);
        blockStride = (nodeCnt + blockNum - 1) / blockNum;
        std::thread threads[NUM_THREAD];
        for (int i = 0; i < NUM_THREAD; ++i) {
            threads[i] = thread(solveThread, i);
        }
        for (auto &thread : threads) {
            thread.join();
        }
    }
}

#ifdef TEST

void displayAnsInfo(int ansNum) {
    printf("Total Loops %d\n", ansNum);
}

#endif

void save_write(const string &outputFile) {
    int fp = open(outputFile.c_str(), O_WRONLY | O_CREAT, 00666);
    int ansNum = 0;

    for (int ti = 0; ti < NUM_THREAD; ++ti)
        ansNum += bundles[ti].ansCnt;

#ifdef TEST
    displayAnsInfo(ansNum);
    int cnt = 0;
#endif
    char buf[12];
    char *ptrEd = buf + 10;
    char *ptr = int2charX2(ansNum, ptrEd);
    *ptrEd = '\n';
    write(fp, ptr, ptrEd - ptr + 1);

    for (int di = 0; di < DEPTH_NUM; ++di) {
        for (int b = 0; b < blockNum; ++b) {
            auto &block = blocks[b];
            int bufferNum = block.bufferCnt[di];
//                printf("Block %d has %d buffer pointers\n",b,bufferNum);
            for (int p = 0; p < bufferNum; p++) {
                char *cur = block.startAddr[di][p];
                char *ed = block.endAddr[di][p];
                if (cur >= ed) continue;
//                    printf("Depth:%d Block %d St:%llu Ed:%llu num:%llu\n",di,b,(ull)cur,(ull)ed,(ull)(ed-cur)/sz);
#ifdef TEST
                ++cnt;
#endif
                write(fp, cur, ed - cur);
            }
        }
    }
    //close(fp);
#ifdef TEST
    printf("Total write times:%d\n", cnt);
#endif
}

void save_fwrite(const string &outputFile) {
    FILE *fp = fopen(outputFile.c_str(), "wb");
    int ansNum = 0;

    for (int ti = 0; ti < NUM_THREAD; ++ti)
        ansNum += bundles[ti].ansCnt;

#ifdef TEST
    displayAnsInfo(ansNum);
#endif

    char buf[12];
    char *ptrEd = buf + 10;
    char *ptr = int2charX2(ansNum, ptrEd);
    *ptrEd = '\n';
    fwrite(ptr, ptrEd - ptr + 1, sizeof(char), fp);

    for (int di = 0; di < DEPTH_NUM; ++di) {
        for (int b = 0; b < blockNum; ++b) {
            auto &block = blocks[b];
            int bufferNum = block.bufferCnt[di];
//                printf("Block %d has %d buffer pointers\n",b,bufferNum);
            for (int p = 0; p < bufferNum; p++) {
                char *cur = block.startAddr[di][p];
                char *ed = block.endAddr[di][p];
                if (cur >= ed) continue;
                if (cur >= ed) continue;
//                    printf("Depth:%d Block %d St:%llu Ed:%llu num:%llu\n",di,b,(ull)cur,(ull)ed,(ull)(ed-cur)/sz);
                fwrite(cur, ed - cur, sizeof(char), fp);
            }
        }
    }
    fclose(fp);
}

int main(int argc, char *argv[]) {
#ifdef CONFIG_USE_MMAP
    //调低进程的友善值
    nice(-20);
#endif

    string testFile;
    if (argc > 1)
        testFile = string(argv[1]);
    else {
#ifdef TEST
//                testFile = "test_data_SFN.N1560268.E200W.A18875018.txt";
//                testFile = "test_data7.fs.txt";
        testFile = "test_data.N111314.E200W.A19630345.txt";
#else
        testFile = "/data/test_data.txt";
#endif
    }

    string outputFile = "/projects/student/result.txt";
#ifdef TEST
    outputFile = "output.txt";
    UniversalTimer timerA, timerB;
    timerA.setTime();
    timerB.setTime();
#endif

    parseInput(testFile);
#ifdef TEST
    timerB.resetTimeWithTag("Read Input File");
#endif

    constructGraph();
#ifdef TEST
    timerB.resetTimeWithTag("Construct Graph");
#endif

    topoSort();
#ifdef TEST
    timerB.resetTimeWithTag("TopoSort");
#endif

    threadingReconstructGraph();

#ifdef TEST
    int edgeCntG=0, edgeCntGInv=0;
    for (int i = 1; i < nodeCnt; ++i) edgeCntG+=G[i].ed-G[i].st, edgeCntGInv+=GInv[i].ed-GInv[i].st;
    printf("[G]-%d [GInv]-%d Edges in Total After TopoSort\n", edgeCntG,edgeCntGInv);
#endif

#ifdef TEST
    timerB.resetTimeWithTag("Reconstruct Graph");
    timerB.printLogs();
#endif

    solveWithThreads();
#ifdef TEST
    timerB.resetTimeWithTag("Solving Results");
#endif

//    save_fwrite(outputFile);
    save_write(outputFile);

#ifdef TEST
    timerB.resetTimeWithTag("Output");
#endif

#ifdef TEST
    timerB.printLogs();
    timerA.logTime("Whole Process");
#endif
    exit(0);
    return 0;
}
