#include <bits/stdc++.h>

using namespace std;

///@Author 小白兔奶糖
///@Date 2020.5
///@Algorithm +4-4

#include <ext/pb_ds/hash_policy.hpp>
#include <ext/pb_ds/assoc_container.hpp>

#define CONFIG_USE_NEON
#define CONFIG_USE_MMAP
//#define TEST

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
    for (i = 0; i <= (int)count; i++) {
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
const int MAX_DEPTH = 8;
const int MIN_DEPTH = 3;
const int DEPTH_NUM = MAX_DEPTH - MIN_DEPTH + 1;
const int MAX_TOTAL_PRE4_PATH = MAXN*2;
const unsigned int MAX_MEMORY_POOL_SIZE = 3u * 1024 * 1024 * 1024; //3G

#define NUM_THREAD 4
#define NUM_BLOCK 1024

//~48M
#define SIZE_PER_ANS_BUFFER  48*1048576


//线上ID不连续
int inputs[MAXE * 2]; //u-v pairs
ull weights[MAXE]; //w
__gnu_pbds::gp_hash_table<int, int> idHash; //remap sorted id to 1...n
int tmpIds[MAXN]; //sorted u
int ids[MAXN]; //sorted u

struct Edge {
    int to;
    ull w;

    Edge() {}

    Edge(int to, ull w) : to(to), w(w) {}

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
#define lp(a) ((a>>32) & 0xffffffff)
#define rp(a) (a & 0xffffffff)

struct P4Info {
    //->pre4[w4]->pre3[w3]->pre2[w2]->pre1[w1]->head
    int pre3,pre2, pre1;
    ull pre32, w1, w4;
    P4Info() {}

    P4Info(int pre3, int pre2, int pre1, ull w1, ull w4) : pre3(pre3), pre2(pre2), pre1(pre1), w1(w1), w4(w4) {
        pre32=mp(pre3,pre2);
    }

    bool operator<(const P4Info &rhs) const {
        if(pre32!=rhs.pre32) return pre32<rhs.pre32;
        return pre1<rhs.pre1;
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
    int ansCnt; //sum of [3,4,5,6,7,8][threadId]
    char *bufferPtr[DEPTH_NUM];
    int bufferSize[DEPTH_NUM];


    ////中间变量
    bool vis[MAXN];

    //->pre4[w4]->pre3[w3]->pre2[w2]->pre1[w1]->head
    //reachableP4[0][pre4]=head indicates pre4->pre3->pre2->pre1->head
    //reachableP4[1][pre4]=addr[pre4] indicates addr of path list
    //reachableP4[2][pre4]=ed indicates end of path list
    int reachableP4[3][MAXN];

    P4Info *pre123List[MAXN];

    //<pre4,pre3,pre2,pre1> should be sorted in ascending order
    //数组模拟链表（链式前向星）存储
    P4Info pre123Buffer[MAX_TOTAL_PRE4_PATH];
    int pre123Head[MAXN]; //store idx of head in buffer
    int pre123Nxt[MAX_TOTAL_PRE4_PATH]; //store next idx of current idx
    int pre4Set[MAXN]; //store all head

    ull ans3[MAXN];
    int ans3Cnt;
    ////

    inline bool checkNode(int i) {
        return G[i].st < G[i].ed && GInv[i].st < GInv[i].ed;
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
int inputPairSize = 0;

void parseInput(string &testFile) {
#ifndef CONFIG_USE_MMAP
    FILE *file = fopen(testFile.c_str(), "r");
    int u, v;
    double fc;
    while (fscanf(file, "%d,%d,%lf", &u, &v, &fc) != EOF) {
        inputs[inputSize++] = u;
        inputs[inputSize++] = v;
        weights[inputPairSize++] = fc*100;
    }
#else
    int fd = open(testFile.c_str(), O_RDONLY);
    int fileLength = lseek(fd, 0, SEEK_END);
    char *buffer =
            (char *) mmap(NULL, fileLength, PROT_READ, MAP_PRIVATE, fd, 0);

    //as buffer reaching bufferEnd
    //value won't be in ['0','9']
    while (*buffer >= '0') {
        int u = 0, v = 0;
        ull w = 0;
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
        w *= 100;
        if (*buffer == '.') {
            ++buffer;
            if (*buffer >= '0') {
                w = w + (*(buffer++) - '0') * 10;
                if (*buffer >= '0') {
                    w = w + *(buffer++) - '0';
                }
            }
        }
        if (*buffer == '\r') ++buffer;
        ++buffer;
        inputs[inputSize++] = u;
        inputs[inputSize++] = v;
        ids[inputPairSize] = u;
        weights[inputPairSize++] = w;
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
    int cnt = inputPairSize;
    if(cnt<20)
        sort(ids, ids + cnt);
    else threadingSortInputs(cnt);

    nodeCnt = unique(ids, ids + cnt) - ids;
#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Sort Inputs");
#endif
    //map to [1,nodeCnt)
    nodeCnt++;
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

    for (int i = 0; i < inputSize; i += 2) {
        auto it=idHash.find(inputs[i + 1]);
        if (it == idHash.end()) {
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

    for (int i = 0; i < inputSize; i += 2) {
        if (inputs[i] < 0) continue;
        int u = inputs[i], v = inputs[i + 1];
        ull w=weights[i>>1];
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

inline bool checkWeight(ull& x, ull& y) {
    //y/x in [0.2,3]
    //return x <= 5ll * y && y <= 3ll * x;
    return (y << 2) + y >= x && (x << 1) + x >= y;
}

int constructP4(int head, Bundle &bundle, Block &block) {
    int pre1 = 0, pre2 = 0, pre3 = 0, pre4=0, p4Cnt = 0, pathCnt = 0;
    ull w1 = 0, w2 = 0, w3 = 0, w4=0;

    auto &ans3=bundle.ans3;
    int ans3Cnt=0;
    //->pre3[w3]->pre2[w2]->pre1[w1]->head
    //head<min(pre1,pre2,pre3)
    //pre2!=pre1 pre3!=pre1 > head is assured here
    //pre2!=pre3 pre1!=pre3 > head and pre2,pre1 not visited is assured in dfs

    auto &pre123Buffer = bundle.pre123Buffer;
    auto &pre123Head = bundle.pre123Head;
    auto &pre123Nxt = bundle.pre123Nxt;
    auto &pre4Set = bundle.pre4Set;

    auto &pre123List = bundle.pre123List;

    auto &reachableP4 = bundle.reachableP4;
    auto &vis = bundle.vis;

    for (int it1 = GInv[head].st; it1 < GInv[head].ed; ++it1) {
        pre1 = edgesGInv[it1].to;
        if (pre1<=head || vis[pre1]) continue;
        w1 = edgesGInv[it1].w;
//        printf("->%d[%d]->%d\n",ids[pre1-1],w1,ids[head-1]);
        for (int it2 = GInv[pre1].st; it2 < GInv[pre1].ed; ++it2) {
            pre2 = edgesGInv[it2].to;
            w2 = edgesGInv[it2].w;
            if (pre2<=head || vis[pre2] || pre2 == pre1 || !checkWeight(w2, w1)) //21
                continue;
//            printf("->%d[%d]->%d[%d]->%d\n",ids[pre2-1],w2,ids[pre1-1],w1,ids[head-1]);
            for (int it3 = GInv[pre2].st; it3 < GInv[pre2].ed; ++it3) {
                pre3 = edgesGInv[it3].to;
                w3 = edgesGInv[it3].w;
                if(pre3<head) continue;
                if(pre3==head){
                    if(checkWeight(w3, w2) && checkWeight(w1, w3)){
                        ans3[ans3Cnt++]=mp(pre2,pre1);
                    }
                }else if (vis[pre3] || pre3 == pre1 || pre3==pre2 || !checkWeight(w3, w2)) //32
                    continue;
//                printf("->%d[%d]->%d[%d]->%d[%d]->%d\n",ids[pre3-1],w3,ids[pre2-1],w2,ids[pre1-1],w1,ids[head-1]);

                for (int it4 = GInv[pre3].st; it4 < GInv[pre3].ed; ++it4) {
                    pre4 = edgesGInv[it4].to;
                    w4 = edgesGInv[it4].w;

                    if(pre4<head) continue;
                    if (pre4 == head) {
                        if (!checkWeight(w1, w4)) continue;
                    } else if (vis[pre4] || pre4 == pre1 || pre4==pre2 || pre4==pre3) continue;
                    if (!checkWeight(w4, w3)) continue;  //43

                    if (reachableP4[0][pre4] != head) {
                        reachableP4[0][pre4] = head;
                        pre123Head[pre4] = -1;
                        pre4Set[p4Cnt++] = pre4;
                    }

                    pre123Buffer[pathCnt] = P4Info(pre3, pre2, pre1, w1, w4);
                    pre123Nxt[pathCnt] = pre123Head[pre4];
                    pre123Head[pre4] = pathCnt++;
                }
            }
        }
    }
    bundle.ans3Cnt=ans3Cnt;

    if (p4Cnt == 0) return 0;
    int lastPos = 0;
    for (int i = 0; i < p4Cnt; ++i) {
        pre4 = pre4Set[i];
        int curPos = lastPos;
        for (int idx = pre123Head[pre4]; idx != -1; idx = pre123Nxt[idx]) {
            pre123List[curPos++] = &pre123Buffer[idx];
        }
        if (curPos - lastPos > 1)
            sort(pre123List + lastPos, pre123List + curPos,
                 [&](const P4Info *lhs, const P4Info *rhs) -> bool { return *lhs < *rhs; });
        reachableP4[1][pre4] = lastPos;
        reachableP4[2][pre4] = curPos;
        lastPos = curPos;
    }
    return p4Cnt;
}

//check [3] in depth [4]
//check [4] in P4
//check [5] in depth [1]
//check [6] in depth [2]
//check [7] in depth [3]
//check [8] in depth [4]
void dfsP3NR(int startIdx, int endIdx, Bundle &bundle, Block &block) {
    auto &vis = bundle.vis;
    auto &reachableP4 = bundle.reachableP4;
    auto &pre123List = bundle.pre123List;
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
        if (!bundle.checkNode(head)) continue;

        //若P4Cnt==0 只可能有长度为3的环
        int p4Cnt=constructP4(head, bundle, block);

        //check loop 3
        int ans3Cnt=bundle.ans3Cnt;
        if(ans3Cnt){
            auto &ans3=bundle.ans3;
            sort(ans3,ans3+ans3Cnt);
            //path3: pre3 pre2 pre1
            for(int idx=0;idx<ans3Cnt;++idx){
                //dump path
                checkBuffer(0);
                auto &bufferPtr = bundle.bufferPtr[0];
                auto &bufferSize = bundle.bufferSize[0];
                auto originBufferPtr = bufferPtr;

                int pre2 = lp(ans3[idx]);
                int pre1 = rp(ans3[idx]);

                memcpy16BX1(bufferPtr, idsComma[head]);
                bufferPtr += idsComma[head][15];
                memcpy16BX1(bufferPtr, idsComma[pre2]);
                bufferPtr += idsComma[pre2][15];
                memcpy16BX1(bufferPtr, idsComma[pre1]);
                bufferPtr += idsComma[pre1][15];

                *(bufferPtr - 1) = '\n';
                bufferSize -= (bufferPtr - originBufferPtr);

                ++ansCnt;
            }
        }

        if(p4Cnt==0) continue;

        //head/f1[w12]->f2[w23]->f3[w34]->f4[w45]->p3[w3]->p2[w2]->p1[w1]->head
        //<w12,w23> <w23,w34> <w34,w45> 在循环中检查
        //<lst,w3> and <w1,w12> 在记录路径时检查
        //<w3,w2> <w2,w1> 在构建P3与P2时检查
        vis[head] = true;

        int curPos, pathAddrEd, pre3, pre2, pre1;

        //check loop 4
        if (reachableP4[0][head] == head) {
            curPos = reachableP4[1][head];
            pathAddrEd = reachableP4[2][head];
            for (; curPos < pathAddrEd; ++curPos) {
                //<pre2,pre1>,<w1,w3>
                auto &pathInfo = *pre123List[curPos];

                //dump path
                checkBuffer(1);
                auto &bufferPtr = bundle.bufferPtr[1];
                auto &bufferSize = bundle.bufferSize[1];
                auto originBufferPtr = bufferPtr;

                pre3 = pathInfo.pre3;
                pre2 = pathInfo.pre2;
                pre1 = pathInfo.pre1;

                memcpy16BX1(bufferPtr, idsComma[head]);
                bufferPtr += idsComma[head][15];
                memcpy16BX1(bufferPtr, idsComma[pre3]);
                bufferPtr += idsComma[pre3][15];
                memcpy16BX1(bufferPtr, idsComma[pre2]);
                bufferPtr += idsComma[pre2][15];
                memcpy16BX1(bufferPtr, idsComma[pre1]);
                bufferPtr += idsComma[pre1][15];

                *(bufferPtr - 1) = '\n';
                bufferSize -= (bufferPtr - originBufferPtr);

                ++ansCnt;
            }
        }

        for (int it1 = G[head].st; it1 < G[head].ed; ++it1) {
            int &f2 = edgesG[it1].to;
            if (f2<=head || vis[f2]) continue;
            ull &w12 = edgesG[it1].w;
            vis[f2] = true;
            //check loop 5
            if (reachableP4[0][f2] == head) {
                curPos = reachableP4[1][f2];
                pathAddrEd = reachableP4[2][f2];
                for (; curPos < pathAddrEd; ++curPos) {
                    auto &pathInfo = *pre123List[curPos];
                    pre3 = pathInfo.pre3;
                    pre2 = pathInfo.pre2;
                    pre1 = pathInfo.pre1;
                    //vis & weight check
                    if (vis[pre1] || vis[pre2] || vis[pre3] ||
                        !checkWeight(w12, pathInfo.w4) ||
                        !checkWeight(pathInfo.w1, w12))
                        continue;
                    //dump path
                    checkBuffer(2);
                    auto &bufferPtr = bundle.bufferPtr[2];
                    auto &bufferSize = bundle.bufferSize[2];
                    auto originBufferPtr = bufferPtr;

                    memcpy16BX1(bufferPtr, idsComma[head]);
                    bufferPtr += idsComma[head][15];
                    memcpy16BX1(bufferPtr, idsComma[f2]);
                    bufferPtr += idsComma[f2][15];
                    memcpy16BX1(bufferPtr, idsComma[pre3]);
                    bufferPtr += idsComma[pre3][15];
                    memcpy16BX1(bufferPtr, idsComma[pre2]);
                    bufferPtr += idsComma[pre2][15];
                    memcpy16BX1(bufferPtr, idsComma[pre1]);
                    bufferPtr += idsComma[pre1][15];

                    *(bufferPtr - 1) = '\n';
                    bufferSize -= (bufferPtr - originBufferPtr);

                    ++ansCnt;
                }
            }

            for (int it2 = G[f2].st; it2 < G[f2].ed; ++it2) {
                int &f3 = edgesG[it2].to;
                ull &w23 = edgesG[it2].w;
                if (f3<=head || !checkWeight(w12, w23) || vis[f3]) continue;
                vis[f3] = true;
                //check loop 6
                if (reachableP4[0][f3] == head) {
                    curPos = reachableP4[1][f3];
                    pathAddrEd = reachableP4[2][f3];
                    for (; curPos < pathAddrEd; ++curPos) {
                        auto &pathInfo = *pre123List[curPos];
                        pre3 = pathInfo.pre3;
                        pre2 = pathInfo.pre2;
                        pre1 = pathInfo.pre1;
                        //vis & weight check
                        if (vis[pre1] || vis[pre2] || vis[pre3] ||
                            !checkWeight(w23, pathInfo.w4) ||
                            !checkWeight(pathInfo.w1, w12))
                            continue;
                        //dump path
                        checkBuffer(3);

                        auto &bufferPtr = bundle.bufferPtr[3];
                        auto &bufferSize = bundle.bufferSize[3];
                        auto originBufferPtr = bufferPtr;

                        memcpy16BX1(bufferPtr, idsComma[head]);
                        bufferPtr += idsComma[head][15];
                        memcpy16BX1(bufferPtr, idsComma[f2]);
                        bufferPtr += idsComma[f2][15];
                        memcpy16BX1(bufferPtr, idsComma[f3]);
                        bufferPtr += idsComma[f3][15];
                        memcpy16BX1(bufferPtr, idsComma[pre3]);
                        bufferPtr += idsComma[pre3][15];
                        memcpy16BX1(bufferPtr, idsComma[pre2]);
                        bufferPtr += idsComma[pre2][15];
                        memcpy16BX1(bufferPtr, idsComma[pre1]);
                        bufferPtr += idsComma[pre1][15];

                        *(bufferPtr - 1) = '\n';
                        bufferSize -= (bufferPtr - originBufferPtr);

                        ++ansCnt;

                    }
                }

                for (int it3 = G[f3].st; it3 < G[f3].ed; ++it3) {
                    int &f4 = edgesG[it3].to;
                    ull &w34 = edgesG[it3].w;
                    if (f4<=head || !checkWeight(w23, w34) || vis[f4]) continue;
                    vis[f4] = true;
                    //check loop 7
                    if (reachableP4[0][f4] == head) {
                        curPos = reachableP4[1][f4];
                        pathAddrEd = reachableP4[2][f4];
                        for (; curPos < pathAddrEd; ++curPos) {
                            auto &pathInfo = *pre123List[curPos];
                            pre3 = pathInfo.pre3;
                            pre2 = pathInfo.pre2;
                            pre1 = pathInfo.pre1;
                            //vis & weight check
                            if (vis[pre1] || vis[pre2] || vis[pre3] ||
                                !checkWeight(w34, pathInfo.w4) ||
                                !checkWeight(pathInfo.w1, w12))
                                continue;
                            //dump path
                            checkBuffer(4);

                            auto &bufferPtr = bundle.bufferPtr[4];
                            auto &bufferSize = bundle.bufferSize[4];
                            auto originBufferPtr = bufferPtr;

                            memcpy16BX1(bufferPtr, idsComma[head]);
                            bufferPtr += idsComma[head][15];
                            memcpy16BX1(bufferPtr, idsComma[f2]);
                            bufferPtr += idsComma[f2][15];
                            memcpy16BX1(bufferPtr, idsComma[f3]);
                            bufferPtr += idsComma[f3][15];
                            memcpy16BX1(bufferPtr, idsComma[f4]);
                            bufferPtr += idsComma[f4][15];
                            memcpy16BX1(bufferPtr, idsComma[pre3]);
                            bufferPtr += idsComma[pre3][15];
                            memcpy16BX1(bufferPtr, idsComma[pre2]);
                            bufferPtr += idsComma[pre2][15];
                            memcpy16BX1(bufferPtr, idsComma[pre1]);
                            bufferPtr += idsComma[pre1][15];

                            *(bufferPtr - 1) = '\n';
                            bufferSize -= (bufferPtr - originBufferPtr);

                            ++ansCnt;
                        }
                    }

                    for (int it4 = G[f4].st; it4 < G[f4].ed; ++it4) {
                        int &f5 = edgesG[it4].to;
                        ull &w45 = edgesG[it4].w;
                        if (f5<=head || reachableP4[0][f5] != head || !checkWeight(w34, w45) || vis[f5]) continue;
                        vis[f5] = true;
                        //check loop 8
                        curPos = reachableP4[1][f5];
                        pathAddrEd = reachableP4[2][f5];
                        for (; curPos < pathAddrEd; ++curPos) {
                            auto &pathInfo = *pre123List[curPos];
                            pre3 = pathInfo.pre3;
                            pre2 = pathInfo.pre2;
                            pre1 = pathInfo.pre1;
                            //vis & weight check
                            if (vis[pre1] || vis[pre2] || vis[pre3] ||
                                !checkWeight(w45, pathInfo.w4) ||
                                !checkWeight(pathInfo.w1, w12))
                                continue;
                            //dump path
                            checkBuffer(5);

                            auto &bufferPtr = bundle.bufferPtr[5];
                            auto &bufferSize = bundle.bufferSize[5];
                            auto originBufferPtr = bufferPtr;

                            memcpy16BX1(bufferPtr, idsComma[head]);
                            bufferPtr += idsComma[head][15];
                            memcpy16BX1(bufferPtr, idsComma[f2]);
                            bufferPtr += idsComma[f2][15];
                            memcpy16BX1(bufferPtr, idsComma[f3]);
                            bufferPtr += idsComma[f3][15];
                            memcpy16BX1(bufferPtr, idsComma[f4]);
                            bufferPtr += idsComma[f4][15];
                            memcpy16BX1(bufferPtr, idsComma[f5]);
                            bufferPtr += idsComma[f5][15];
                            memcpy16BX1(bufferPtr, idsComma[pre3]);
                            bufferPtr += idsComma[pre3][15];
                            memcpy16BX1(bufferPtr, idsComma[pre2]);
                            bufferPtr += idsComma[pre2][15];
                            memcpy16BX1(bufferPtr, idsComma[pre1]);
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
    auto &vis = bundles[threadId].vis;
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
                testFile = "test_data2.txt";
//            testFile = "test_data_k14float.txt";
//                testFile = "test_data.fsb.txt";
//        testFile = "test_data.N111314.E200W.A19630345.txt";
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
