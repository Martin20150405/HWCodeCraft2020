#include <bits/stdc++.h>

using namespace std;

#define CONFIG_USE_NEON
#define CONFIG_USE_MMAP

#ifdef CONFIG_USE_NEON
#include <arm_neon.h>
#endif
#ifdef CONFIG_USE_MMAP

#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#endif

//#define TEST

///Timer
#ifdef TEST
#ifdef _WIN32   // windows

#include <sysinfoapi.h>

#else   // unix

#include <sys/time.h>

#endif

struct UniversalTimer {
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

    void logTime(string tag) {
        printf("Time consumed for %s is %d ms.\n", tag.c_str(), getElapsedTimeMS());
    }
};

int sumOver(int s[], int n) {
    int ret = 0;
    for (int i = 0; i < n; i++) ret += s[i];
    return ret;
}

#endif
///

#ifdef CONFIG_USE_NEON
//128bit 16B one op
//will copy 16 if [1,16)
void memcpy16B(void *dest, void *src, size_t count){
    int i;
    unsigned long *s = (unsigned long *)src;
    unsigned long *d = (unsigned long *)dest;
    count /= 16;
    for (i = 0; i <= count; i++) {
        vst1q_u64(&d[0], vld1q_u64(&s[0]));
        d += 2; s += 2;
    }
}

#endif

typedef long long ll;
//maxN=560000
//maxE=280000 ~avgN=28000
const int MAXE = 280000;
const int MAXN = 262144;
const int MAX_LOOP = 3000000;
const int MAX_DEPTH = 7;
const int MAX_OUT_DEGREE = 52;
const int MAX_IN_DEGREE = 52;
const int MAX_PRE2 = 50 * 5 + 2;
const int MAX_PRE3 = 50 * 60;

#define NUM_THREAD 4
#define NUM_BLOCK 512

//线上ID不连续，但是最大不超过26W
int inputs[MAXE * 2]; //u-v pairs
bool inputsVis[MAXN];
int idHash[MAXN]; //sorted id to 1...n

//maxN=560000
//maxE=280000 ~avgN=28000

int G[MAXN][MAX_OUT_DEGREE];
int gSize[MAXN];

int GInv[MAXN][MAX_IN_DEGREE];
int gInvSize[MAXN];

char idsComma[MAXN][16]; //最大长度为11，长度10+逗号/换行符,Length at start

int inDegrees[MAXN];
int outDegrees[MAXN];
int que[MAXN];

////答案存储
struct Answer {
    char a3[401408];    //0.4M
    char a4[8003584];   //8M
    char a5[15003648];  //15M
    char a6[30007296];  //30M
    char a7[58003456];  //58M
    char *addr[6];

    Answer() {
        addr[0] = a3;
        addr[1] = a4;
        addr[2] = a5;
        addr[3] = a6;
        addr[4] = a7;
    }
};

struct Block {
    //thread i use ans[i]'s buffer
    char *startAddr[6];
    int length[5];
};

bool blockVis[NUM_BLOCK];

//4x113M
Answer ans[NUM_THREAD];
Block blocks[NUM_BLOCK];
int blockNum;
int blockStride;
int currentBlock;
mutex currentBlockLock;  //unique lock
//int ans[5][MAX_LOOP][MAX_DEPTH];   //[3,4,5,6,7]
int ansCnt[NUM_THREAD][6];
int ansIdx[NUM_THREAD][5];
char pathBuffer[NUM_THREAD][80];
int pathBufferIdx[NUM_THREAD][MAX_DEPTH+1];
////

////中间变量
bool vis[NUM_THREAD][MAXN];
//reachableP1[pre1]=head indicates pre1->head
int reachableP1[NUM_THREAD][MAXN];
//reachableP2[pre2][0]=head indicates pre2->pre1->head
//reachableP2[pre2][1]=addr[pre2] indicates addr of pre1 list
int reachableP2[NUM_THREAD][MAXN][2];
//pre1List[addr] stores all pre1 leading to head
//DIM1: pre2AddrList DIM2:pre1 List (size in pos 0)
int pre1List[NUM_THREAD][MAX_PRE2][MAX_IN_DEGREE];

//reachableP3[pre3][0]=head indicates pre3->pre2->pre1->head
//reachableP3[pre3][1]=addr[pre3] indicates addr of <pre1,pre2> list
int reachableP3[NUM_THREAD][MAXN][2];
//sorted pre2 when finding <pre1,pre2> pair
int pre2Set[NUM_THREAD][MAX_PRE2];
ll pre12List[NUM_THREAD][MAX_PRE3][MAX_IN_DEGREE * MAX_IN_DEGREE];
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
        inputsVis[u] = true;
    }
#else
    struct stat fs;
    int fd = open(testFile.c_str(), O_RDONLY);
    fstat(fd, &fs);
    int fileLength = fs.st_size;
    char *buffer =
            (char *) mmap(NULL, fileLength, PROT_READ, MAP_PRIVATE, fd, 0);
    int cur = 0, idx = 0;
    char c;
    bool flag = false;
    while (idx < fileLength) {
        c = *(buffer + idx);
        if (c >= '0') {
            cur = cur * 10 + c - '0';
        } else { //',' or '\r\n'
            inputs[inputSize++] = cur;
            if (flag) { //pos 1 skip pos 2
                while (buffer[++idx] != '\n');
                flag = false;
            } else { //pos 0
                flag = true;
                inputsVis[cur] = true;
            }
            cur = 0;
        }
        ++idx;
    }
#endif
}

void constructGraph() {

    //update nodeCnt idsComma idsLF ids idHash G
    //update inDegrees outDegrees

    //map to [1,nodeCnt)
    nodeCnt = 1;

    char buf[12];
    char *ptrEd = buf + 11;
    for (int id = 0; id < MAXN; ++id) {
        if (inputsVis[id]) {
            idHash[id] = nodeCnt;

            int tmp = id;
            char *ptr = ptrEd;
            do {
                *(--ptr) = tmp % 10 + '0';
                tmp /= 10;
            } while (tmp);

            *ptrEd = ',';
            char len = ptrEd - ptr + 1;
            memcpy(idsComma[nodeCnt] + 1, ptr, len);
            idsComma[nodeCnt][0] = len;

            ++nodeCnt;
        }
    }

#ifdef TEST
    printf("%d Nodes in Total\n", nodeCnt-1);
#endif

    for (int i = 0; i < inputSize; i += 2) {
        if (!inputsVis[inputs[i + 1]]) continue;
        int u = idHash[inputs[i]], v = idHash[inputs[i + 1]];
        G[u][gSize[u]++] = v;
        GInv[v][gInvSize[v]++] = u;

        ++inDegrees[v];
        ++outDegrees[u];
    }
}

void topoSort() {
    int queueIdx = -1;
    for (int i = 1; i < nodeCnt; i++) {
        if (0 == inDegrees[i])
            que[++queueIdx] = i;
    }
    while (queueIdx >= 0) {
        int u = que[queueIdx--];
        for (int i = 0; i < gSize[u]; ++i) {
            int v = G[u][i];
            --outDegrees[u];
            if (0 == --inDegrees[v]) {
                que[++queueIdx] = v;
            }
        }
    }

    for (int i = 1; i < nodeCnt; i++) {
        if (0 == outDegrees[i])
            que[++queueIdx] = i;
    }
    while (queueIdx >= 0) {
        int v = que[queueIdx--];
        for (int i = 0; i < gInvSize[v]; ++i) {
            int u = GInv[v][i];
            --inDegrees[v];
            if (0 == --outDegrees[u]) {
                que[++queueIdx] = v;
            }
        }
    }

}

void sortGraph(int st, int ed) {
    for (int i = st; i < ed; ++i) {
        if (inDegrees[i] == 0 || outDegrees[i] == 0) {
            gSize[i] = 0;
            gInvSize[i] = 0;
        } else {
            sort(G[i], G[i] + gSize[i]);
            sort(GInv[i], GInv[i] + gInvSize[i]);
        }
    }
}

inline void threadingSort() {
    if (nodeCnt < 100) sortGraph(1, nodeCnt);
    else {
        int segmentSize = nodeCnt >> 2;
        int startCnt = 1;
        int endCnt = segmentSize;
        auto t1 = thread(sortGraph, startCnt, endCnt);

        startCnt = endCnt;
        endCnt += segmentSize;
        auto t2 = thread(sortGraph, startCnt, endCnt);

        startCnt = endCnt;
        endCnt += segmentSize;
        auto t3 = thread(sortGraph, startCnt, endCnt);

        startCnt = endCnt;
        endCnt = nodeCnt;
        auto t4 = thread(sortGraph, startCnt, endCnt);
        t1.join();
        t2.join();
        t3.join();
        t4.join();
    }
}

inline void dumpPath(int depth, int threadId, Block &block) {
    int curDepthMinus3 = depth - 3;
#ifdef CONFIG_USE_NEON
    memcpy16B(block.startAddr[curDepthMinus3] + block.length[curDepthMinus3], pathBuffer[threadId],
              pathBufferIdx[threadId][depth]);
#else
    memcpy(block.startAddr[curDepthMinus3] + block.length[curDepthMinus3], pathBuffer[threadId],
           pathBufferIdx[threadId][depth]);
#endif
    ansIdx[threadId][curDepthMinus3] += pathBufferIdx[threadId][depth];
    block.length[curDepthMinus3] += pathBufferIdx[threadId][depth];
    block.startAddr[curDepthMinus3][block.length[curDepthMinus3] - 1] = '\n';
    ++ansCnt[threadId][curDepthMinus3];
}

void checkL3(int head, int threadId, Block &block) {
    //head->nxtId->pre1->head
    int pre1;

    memcpy(pathBuffer[threadId], idsComma[head] + 1, idsComma[head][0]);
    pathBufferIdx[threadId][1] = idsComma[head][0];

    vis[threadId][head] = true;
    auto &gCur = G[head];
    int lim = gSize[head];
    for (int it = 0; it < lim; ++it) {
        int nxtId = gCur[it];
        if (reachableP2[threadId][nxtId][0] == head) {
            //nxtId(pre2)->pre1->head
            memcpy(pathBuffer[threadId]+pathBufferIdx[threadId][1], idsComma[nxtId] + 1, idsComma[nxtId][0]);
            pathBufferIdx[threadId][2] = pathBufferIdx[threadId][1] + idsComma[nxtId][0];

            auto &pre1s = pre1List[threadId][reachableP2[threadId][nxtId][1]];
            int sz = pre1s[0];
            //printf("head:%d pre2:%d pre1Size:%d\n",head,nxtId,sz);
            for (int idx = 1; idx <= sz; ++idx) {
                pre1 = pre1s[idx];
                if (vis[threadId][pre1])
                    continue;
                memcpy(pathBuffer[threadId]+pathBufferIdx[threadId][2], idsComma[pre1] + 1, idsComma[pre1][0]);
                pathBufferIdx[threadId][3] = pathBufferIdx[threadId][2] + idsComma[pre1][0];
                dumpPath(3, threadId, block);
            }
        }
    }
    vis[threadId][head] = false;
}

void dfsP2P3(int cur, int depth, int head, int threadId, Block &block) {

    vis[threadId][cur] = true;

    memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth-1], idsComma[cur] + 1, idsComma[cur][0]);
    pathBufferIdx[threadId][depth] = pathBufferIdx[threadId][depth-1] + idsComma[cur][0];

    //handle [3,4]
    if (depth >= 3 && reachableP1[threadId][cur] == head) {
        dumpPath(depth, threadId, block);
    }

    auto &gCur = G[cur];
    for (int it = 0; it < gSize[cur]; ++it) {
        int nxtId = gCur[it];
        if (!vis[threadId][nxtId] && nxtId > head && gSize[nxtId] > 0) {
            //for depth in [1,3]
            //go deeper
            if (depth < 4) {
                dfsP2P3(nxtId, depth + 1, head, threadId, block);
                continue;
            }
            int pre1, pre2;
            //for depth 4
            //check [5]
            if (reachableP1[threadId][nxtId] == head) {
                memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth], idsComma[nxtId] + 1, idsComma[nxtId][0]);
                pathBufferIdx[threadId][depth+1] = pathBufferIdx[threadId][depth] + idsComma[nxtId][0];
                dumpPath(depth + 1, threadId, block);
            }
            //check [6]
            if (reachableP2[threadId][nxtId][0] == head) {
                //nxtId(pre2)->pre1->head
                //pre1 is naturally sorted
                //no self-loop so vis[nxtId]=true; not need
                memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth], idsComma[nxtId] + 1, idsComma[nxtId][0]);
                pathBufferIdx[threadId][depth+1] = pathBufferIdx[threadId][depth] + idsComma[nxtId][0];

                auto &pre1s = pre1List[threadId][reachableP2[threadId][nxtId][1]];
                int sz = pre1s[0];
                //printf("head:%d pre2:%d pre1Size:%d\n",head,nxtId,sz);
                for (int idx = 1; idx <= sz; ++idx) {
                    pre1 = pre1s[idx];
                    if (vis[threadId][pre1])
                        continue;
                    memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth+1], idsComma[pre1] + 1, idsComma[pre1][0]);
                    pathBufferIdx[threadId][depth+2] = pathBufferIdx[threadId][depth+1] + idsComma[pre1][0];
                    dumpPath(depth + 2, threadId, block);
                }
            }

            //check [7]
            if (reachableP3[threadId][nxtId][0] == head) {
                //nxtId(pre3)->pre2->pre1->head
                //pre2->pre1 is naturally sorted
                vis[threadId][nxtId] = true;
                memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth], idsComma[nxtId] + 1, idsComma[nxtId][0]);
                pathBufferIdx[threadId][depth+1] = pathBufferIdx[threadId][depth] + idsComma[nxtId][0];
                auto &pre12s = pre12List[threadId][reachableP3[threadId][nxtId][1]];
                int sz = pre12s[0];
                //                printf("head:%d cur:%d pre3:%d pre12Size:%d\n",head,cur,nxtId,sz);
                for (int idx = 1; idx <= sz; ++idx) {
                    pre1 = pre12s[idx] >> 32;
                    pre2 = (pre12s[idx]) & (0xffffffff);
                    if (vis[threadId][pre1] || vis[threadId][pre2])
                        continue;
                    memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth+1], idsComma[pre2] + 1, idsComma[pre2][0]);
                    pathBufferIdx[threadId][depth+2] = pathBufferIdx[threadId][depth+1] + idsComma[pre2][0];

                    memcpy(pathBuffer[threadId] + pathBufferIdx[threadId][depth+2], idsComma[pre1] + 1, idsComma[pre1][0]);
                    pathBufferIdx[threadId][depth+3] = pathBufferIdx[threadId][depth+2] + idsComma[pre1][0];

                    dumpPath(depth + 3, threadId, block);
                }
                vis[threadId][nxtId] = false;
            }
        }
    }
    vis[threadId][cur] = false;
}

inline int constructP1P2(int head, int threadId) {
    int p2Cnt = 0, pre1 = 0, pre2 = 0, pre2Addr;
    //可以通过大于head的id 1步返回的
    auto &gInv1 = GInv[head];
    for (int it1 = 0; it1 < gInvSize[head]; ++it1) {
        pre1 = gInv1[it1];
        if (pre1 <= head || gInvSize[pre1] == 0)
            continue;
        reachableP1[threadId][pre1] = head;
        //可以通过大于head的id 2步返回的
        //pre2->pre1->head
        //head<min(pre1,pre2)
        auto &gInv2 = GInv[pre1];
        for (int it2 = 0; it2 < gInvSize[pre1]; ++it2) {
            //printf("%d->%d->%d\n",gInv2[it2],pre1,head);
            pre2 = gInv2[it2];

            //if (pre2 <= head || gInvSize[pre2] == 0 || pre2==pre1)
            if (pre2 <= head || gInvSize[pre2] == 0)
                continue;

            if (reachableP2[threadId][pre2][0] != head) {
                reachableP2[threadId][pre2][0] = head;
                reachableP2[threadId][pre2][1] = p2Cnt;
                pre2Set[threadId][p2Cnt] = pre2;
                pre1List[threadId][p2Cnt++][0] = 0;
            }
            pre2Addr = reachableP2[threadId][pre2][1];
            pre1List[threadId][pre2Addr][++pre1List[threadId][pre2Addr][0]] = pre1;
        }
    }
    return p2Cnt;
}

inline bool constructP3(int p2Cnt, int head, int threadId) {
    int pre2 = 0, pre3 = 0, pre3Addr, p3Cnt = 0;
    ll pre1;
    sort(pre2Set[threadId], pre2Set[threadId] + p2Cnt);
    //pre3->pre2->pre1->head
    //pre2!=pre1 > head is assured in construct P2
    //pre3!=pre1 > head is assured in construct P3
    //pre3!=pre2 pre3!=pre1 > head is assured in dfs
    //construct pre21List using pre2Set
    //pre1 is extracted from pre1List

    //if p3Cnt==0, maxDepth==3
    for (int it2 = 0; it2 < p2Cnt; ++it2) {
        pre2 = pre2Set[threadId][it2];
        auto &gInv3 = GInv[pre2];
        for (int it3 = 0; it3 < gInvSize[pre2]; ++it3) {
            pre3 = gInv3[it3];
            if (pre3 <= head || gInvSize[pre3] == 0)
                continue;
            if (reachableP3[threadId][pre3][0] != head) {
                reachableP3[threadId][pre3][0] = head;
                reachableP3[threadId][pre3][1] = p3Cnt;
                pre12List[threadId][p3Cnt++][0] = 0;
            }
            pre3Addr = reachableP3[threadId][pre3][1];
            auto &pre1s = pre1List[threadId][reachableP2[threadId][pre2][1]];
            int sz = pre1s[0];
            for (int idx = 1; idx <= sz; ++idx) {
                if (pre1s[idx] != pre3){
                    pre1 = pre1s[idx];
                    //printf("%d->%d->%d->%d\n",pre3,pre2,pre1,head);
                    pre12List[threadId][pre3Addr][++pre12List[threadId][pre3Addr][0]] = (pre1 << 32) | pre2;
                }
            }
        }
    }
    return p3Cnt;
}

void solveBlock(int startBid, int endBid, int threadId) {
    int startIdx = startBid * blockStride;
    int endIdx = endBid * blockStride;
    endIdx = min(nodeCnt, endIdx);
    if (endIdx <= startIdx) return;

#ifdef TEST
//    printf("[Thread %d] is solving Block [%d,%d):[%d,%d)\n", threadId, startBid,endBid, startIdx, endIdx);
//    UniversalTimer timer;
//    timer.setTime();
#endif
    //init block
    Block &block = blocks[startBid];
    blockVis[startBid] = true;
    block.startAddr[0] = ans[threadId].addr[0] + ansIdx[threadId][0];
    block.startAddr[1] = ans[threadId].addr[1] + ansIdx[threadId][1];
    block.startAddr[2] = ans[threadId].addr[2] + ansIdx[threadId][2];
    block.startAddr[3] = ans[threadId].addr[3] + ansIdx[threadId][3];
    block.startAddr[4] = ans[threadId].addr[4] + ansIdx[threadId][4];

    for (int i = startIdx; i < endIdx; ++i) {
        if (gSize[i] > 0 && gInvSize[i] > 0) {
            int p2Cnt = constructP1P2(i, threadId);
            if (p2Cnt > 0) {
                if (constructP3(p2Cnt, i, threadId) > 0) {
                    dfsP2P3(i, 1, i, threadId, block);
                } else {
                    checkL3(i, threadId, block);
                }
            }
        }
    }
#ifdef TEST
//    timer.logTime("Current Block");
#endif
}

void solveThread(int threadId) {
    while (currentBlock < blockNum) {
        int startBid, endBid;
        currentBlockLock.lock();
        startBid = currentBlock;
        int rate = blockNum / (currentBlock+1);
        int stride = 0;
        if (rate >= 8) stride = 1;
        else if (rate >= 4) stride = 2;
        else if (rate >= 2) stride = 4;
        else stride = 8;
        endBid = startBid + stride;
        currentBlock = endBid;
        currentBlockLock.unlock();
        solveBlock(startBid, endBid, threadId);
    }

    for (int i = 0; i < 5; i++)
        ansCnt[threadId][5] += ansCnt[threadId][i];
#ifdef TEST
    int tmp = sumOver(ansCnt[threadId], 5);
    for (int i = 0; i < 5; i++) {
        printf("[Thread %d] Loop Size %d: %d/%d ~ %.3lf\n", threadId, i + 3, ansCnt[threadId][i], tmp,
               ansCnt[threadId][i] * 1.0 / tmp);
    }
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
        vector<thread> threads;
        for (int i = 0; i < NUM_THREAD; ++i) {
            threads.emplace_back(thread(solveThread, i));
        }
        for (int i = 0; i < NUM_THREAD; ++i) {
            threads[i].join();
        }
    }
}

void save_fwrite(const string &outputFile) {
    FILE *fp = fopen(outputFile.c_str(), "wb");
    int ansNum = 0;
    for (int i = 0; i < NUM_THREAD; ++i)
        ansNum += ansCnt[i][5];

#ifdef TEST
    for (int i = 0; i < 5; i++) {
        int tmp = 0;
        for (int j = 0; j < NUM_THREAD; ++j) {
            tmp += ansCnt[j][i];
        }
        printf("[Total] Loop Size %d: %d/%d ~ %.3lf\n", i + 3, tmp, ansNum,
               tmp * 1.0 / ansNum);
    }
    printf("Total Loops %d\n", ansNum);
#endif

    char buf[1024];
    int idx = sprintf(buf, "%d\n", ansNum);
    buf[idx] = '\0';
    fwrite(buf, idx, sizeof(char), fp);
    for (int i = 0; i < 5; ++i) {
        for (int b = 0; b < blockNum; ++b) {
            if (blockVis[b])
                fwrite(blocks[b].startAddr[i], blocks[b].length[i], sizeof(char), fp);
        }
    }
    fclose(fp);
}

int main(int argc, char *argv[]) {

    string testFile;

    if (argc > 1)
        testFile = string(argv[1]);
    else {
#ifdef TEST
        testFile = "test_data2.txt";
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
    timerB.logTime("Read Input File");
    timerB.setTime();
#endif

    constructGraph();
#ifdef TEST
    timerB.logTime("Construct Graph");
    timerB.setTime();
#endif

    topoSort();
    threadingSort();
#ifdef TEST
    timerB.logTime("TopoSort");
    timerB.setTime();
#endif

    solveWithThreads();

#ifdef TEST
    timerB.logTime("Solving Results");
    timerB.setTime();
#endif

    save_fwrite(outputFile);

#ifdef TEST
    timerB.logTime("Output");
    timerB.setTime();
#endif

#ifdef TEST
    timerA.logTime("Whole Process");
#endif
    exit(0);
    return 0;
}
