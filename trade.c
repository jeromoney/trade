#include <stdio.h>
#include <cs50.h>

#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <assert.h>

typedef struct node{
    struct node* prevpath;
    long long unsigned int time;
    int* state;
    long long unsigned int fScore;
    
    // DEBUG
    int trade;
    long long unsigned int trade_cost;
    // END DEBUG
}
node;


typedef struct nodeList{
    struct node* node;
    struct nodeList* next;
}
nodeList;


const bool DEV_MODE = true;

// remove
nodeList* removeItem(nodeList* list , node* current){
    nodeList * traverse = list;
    nodeList * prev = NULL;
    while (traverse != NULL){
        if (traverse -> node == current){
            // we found the node
            // we should be able to remove these conditions to speed up the code
            if ((prev == NULL) && (traverse -> next == NULL)){
                // list is only only one item
                assert(list -> next == NULL);
                return NULL;
            }
            else if ((prev == NULL) && (traverse -> next != NULL)){
                // item is at the beginining of the list with more than one item
                list = list -> next;
                free(traverse);
                return list;
            }
            else if ((prev != NULL) && (traverse -> next != NULL)){
                // item is in the middle of the list
                prev -> next = traverse -> next;
                free(traverse);
                return list;
            }
            else if ((prev != NULL) && (traverse -> next == NULL)){
                // item is at end of the list
                prev -> next = NULL;
                free(traverse);
                return list;
                }
            }
        prev = traverse;
        traverse = traverse -> next;
    }
    // This means that we didn't find the node
    assert(false);
}



// add node to list
 nodeList* add(nodeList * list , node * current){
    nodeList* head = malloc(sizeof(nodeList));
    if (list == NULL){
        head -> node = current;
        head -> next = NULL;
    }
    else{
        head -> node = current;
        head -> next = list;
        }
    return head;

}

// verifies if node is in list. A node should be in the list only once
bool verifyMembership(nodeList * list , node * current){
        // verify current is on open set, once and only once. throw a flag if found twice or more.
        nodeList * traverse = list;
        int count = 0;
        while(traverse != NULL){
            if (current == traverse -> node){
                count++;
            }
            traverse = traverse -> next;
        }
    assert(count<=1);
    // other wise list should have item only once.
    return (count == 1);
}

// verifies that the structure of the list is good
bool verifyStructure(nodeList * list){
    nodeList * traverse = list;
    while(traverse != NULL){
        assert(traverse != traverse -> next);
        traverse = traverse -> next;
    }
    return true;
}

/** Heuristic used in A* search. The lowest estimation of the trade time to get to the goal state. This is composed of the array 
 * subtraction, summed, divide by 3, times the lowest trade time.
 **/
 long long unsigned int  heuristic(int start[] , int goal[] ,  long long unsigned int  minTradeTime , int I){
    
    long long unsigned int  heuristicVal = 0;
    for (int i = 0; i < I ; i++){
        heuristicVal = heuristicVal + abs(start[i] - goal[i]);
    }
    
    // A single trade can get 3 places closer to the goal.
    if ((heuristicVal % 3) > 0 ){
        heuristicVal = heuristicVal / 3 + 1;
    }
    else {
         heuristicVal = heuristicVal / 3;
    }

    heuristicVal = heuristicVal * minTradeTime;
    
    return heuristicVal;
}

long long unsigned int  minTradeTimef(long long unsigned int timeArray[] , int K){
    long long unsigned int minTradeTime = ULLONG_MAX;
    for (int i = 0; i < K; i++)
        if (timeArray[i] < minTradeTime) minTradeTime = timeArray[i];
    // might be a hackish choice and break admissibility.s
    if (minTradeTime == 0) minTradeTime = 1;
    return minTradeTime;
}

long long unsigned int maxTradeTimef(long long unsigned int timeArray[] , int K){
    long long unsigned int maxTradeTime = 0;
    for (int i = 0; i < K; i++)
        if (timeArray[i] > maxTradeTime) maxTradeTime = timeArray[i];
    return maxTradeTime;
}


// compares two states to see if equal
bool cmpstates(int current[] , int goal[] ,  int I){
    for (int i = 0; i < I; i++){
        if (current[i] != goal[i]) return false;
    }
    return true;
}



void aStar(int start[] , int goal[] , long long unsigned int minTradeTime , int I , int** tradeArray , int K , int N , long long unsigned int timeArray[] , long long unsigned int maxTradeTime){
    

    // intialize empty closedSet
    nodeList * closedSet = NULL;
    
    // initalize openSet
    node startNode = {.prevpath = NULL , .time = 0 , .state = start , .fScore = heuristic(start, goal, minTradeTime , I) , .trade = 0 , .trade_cost = 0};

    
    nodeList * openSet = malloc(sizeof(nodeList));
    openSet -> node = &startNode;
    openSet -> next = NULL;
    
    
    //int count = 0;
    while (openSet != NULL){
        //DEBUG
        //int dfdsf = GetInt();
        // //dfdsf =3;
        // printf("generation: %i\n\n" , count);
        // nodeList* test = openSet;
        // int opencount = 0;
        // while (test != NULL){
        //     opencount++;
        //     test = test -> next;
        // }
        //  test = closedSet;

        //  int closedcount = 0;
        // while (test != NULL){
        //     closedcount++;
        //     test = test -> next;
        // }
        // printf("Open Set: %i   Closed Set: %i\n" , opencount, closedcount);
        
        // count++;
        // END DEBUG
        

        // find node in open set with the lowest fScore
        long long unsigned int minfScore = ULLONG_MAX;
        node* current = NULL;
        nodeList* traverse = openSet;
        while (traverse != NULL){
            // choosing node with lowest fScore
            if (traverse -> node -> fScore <= minfScore){
                current = traverse -> node;
                minfScore = current -> fScore;
            }
            traverse = traverse -> next;
        }    


            // DEBUG        
            // for (int i = 0; i < I ; i++){
            //     printf ("%i ", current -> state[i]);
            // }
            // printf("\n");
            // END DEBUG


            
        
        // found solution
        if (cmpstates(current -> state, goal , I)){
            printf("%lli\n" , current -> time);
            
            // DEBUG
        //     printf("FOUND GOAL!!\n");
        //     while (current != NULL){
        //     printf("cost: %lli trade: %i   :::" , current -> trade_cost, current -> trade );
        //     for (int i = 0; i < I ; i++){
        //         printf ("%i ", current -> state[i]);
        //     }
        //     printf("\n");
        //     current = current -> prevpath;
        // }
        //     // END DEBUG
            return;
            //return reconstruct_path(cameFrom, current);
            // use function to calculate total trade time
        }
        

        openSet = removeItem(openSet, current);
        closedSet = add(closedSet , current);

    
        //find neighbors of current
        // the maximum neighbors is the number of trades
        // we need to loop over trades. This might be a costly operation
        // a neighbor of a state requires a positive value in the trade array to match with one of the trades
        for (int i = 0 ; i < K ; i++){
            int tradeItem = tradeArray[i][0];
            int tradeItem1 = tradeArray[i][1];
            int tradeItem2 = tradeArray[i][2];
            //DEBUG
            if (((current -> state[tradeItem1]) + (current -> state[tradeItem2]) > 1) &&
            ((current -> state[tradeItem1]) > 0) &&
            ((current -> state[tradeItem2]) > 0))
            {
                // found a trade that will create a neighbor
                
                int* neighborArray = malloc( I * sizeof(int));

                memcpy(neighborArray, current -> state, I * sizeof(int) );
                assert(neighborArray != current -> state);

                

                neighborArray[tradeItem] = neighborArray[tradeItem] + 1;
                if (neighborArray[tradeItem] < 0){
                    ;
                    assert(neighborArray[tradeItem] >= 0);
                }
                neighborArray[tradeItem1] = neighborArray[tradeItem1] - 1;
                neighborArray[tradeItem2] = neighborArray[tradeItem2] - 1;
                
                int sumOfArray = 0;
                for (int someIndex = 0; someIndex < I ; someIndex++){
                    sumOfArray = sumOfArray + neighborArray[someIndex];
                }
                
                
                
                // Is neighbor in closed set or more than N items?
                traverse = closedSet;
                bool flag = false;
                while (traverse != NULL) {

                    if (cmpstates(neighborArray , traverse -> node -> state , I) || sumOfArray < 1){
                        
                        // neighbor in closed set
                        free(neighborArray);
                        flag = true;
                        break;
                    }
                    traverse = traverse -> next;;
                    if (traverse != NULL) assert(traverse != traverse -> next);
                }
                if (flag){
                ;
                }
                else {
                    
                    long long unsigned int tentative_gScore = current -> time + timeArray[i];
                    // if neighbor is not in open set, add neighbor
                    traverse = openSet;
                    bool notInOpenSet = true;
                    while (traverse!= NULL) {
                        if (cmpstates(neighborArray , traverse -> node -> state , I)){
                            // neighbor in open set?
                            notInOpenSet = false;
                            if (tentative_gScore < traverse -> node -> time){
                                //  record better path
                                traverse -> node -> prevpath = current;
                                traverse -> node -> time = tentative_gScore;
                                traverse -> node -> fScore = traverse -> node -> time + heuristic(neighborArray, goal, minTradeTime , I);
                                
                                // DEBUG
                                 traverse -> node -> trade = i;
                                 traverse -> node -> trade_cost = timeArray[i];
                                
                             
                                // END DEBUG
                            }
                            break;
                        }
                        traverse = traverse -> next;
                    }
                    long long unsigned int heuristicVal = heuristic(neighborArray, goal, minTradeTime , I);
                    long long unsigned int fScore = tentative_gScore + heuristicVal;

                    if ((notInOpenSet) &&  (fScore <= maxTradeTime * (N - 1))){
                    //if ((notInOpenSet) && (fScore <= maxTradeTime * (N - 1))){
                        node * neighborNode = malloc(sizeof(node));
                        neighborNode -> state = neighborArray;
                        neighborNode -> prevpath = current;
                        neighborNode -> time = tentative_gScore;
                        neighborNode -> fScore =fScore;
                        
                        //DEB                                 
                        neighborNode -> trade = i;
                        neighborNode -> trade_cost = timeArray[i];
                        
                        // END DEBUG
    
                        openSet = add(openSet, neighborNode);

                    }
                }
            }
        }
    }
    
    
    // return failure if here 
    printf("-1\n");
    return;
    
}



void aStarF(void){
    
 
    // Read inputs
    // The first line will be an integer, I, the number of item ids (including the item with id 0, the red paperclip).
    int I = GetInt();

    // The next line will be an integer, N, the number of desired items.
    int N = GetInt();

    // The next N lines will contain one desired item id, D, each.
    // Get into an int array
    int desired[N];
    memset( desired, -1, N * sizeof(int) );
    for (int i = 0; i < N; i++) desired[i] = GetInt();

    // The next line will be an integer, K, the number of trade offers.
    int K = GetInt();

    // The next K lines will contain one trade offer each. Each trade offer will be a space-separated sequence of integers, with the following format
    int** tradeArray;
    tradeArray = malloc(K * sizeof(int*));
    // might be hackish and I should just declare size 3
    for (int i = 0; i < K; i++) tradeArray[i] = malloc(K * sizeof(int));
    for (int i = 0; i < K; i++) for (int j = 0; j < K; j++) tradeArray[i][j] = -1;
    long long unsigned int timeArray[K];
    memset( timeArray, 0, K * sizeof(long long unsigned int) );
    
    
    for (int i = 0; i < K; i++){
        long long unsigned int time;
        int item1;
        int item2;
        int item3;
        string str = GetString();
        sscanf(str, "%lli %i %i %i\n", &time, &item1, &item2, &item3);
        
        
        // need to add array that holds trades.
        timeArray[i] = time;
        tradeArray[i][0] = item1;
        tradeArray[i][1] = item2;
        tradeArray[i][2] = item3;
        } 
    //}
    
 
    // Eliminate duplicate trades
    long long unsigned int atime;
    int aitem1;
    int aitem2;
    int aitem3;

    long long unsigned int btime;
    int bitem1;
    int bitem2;
    int bitem3;    
    
    for (int i = 0; i < K - 1; i++){
        atime  =  timeArray[i];
        aitem1 =  tradeArray[i][0];
        aitem2 =  tradeArray[i][1];
        aitem3 =  tradeArray[i][2];
        
        for (int j = i + 1; j < K; j++){
            btime  =  timeArray[j];
            bitem1 =  tradeArray[j][0];
            bitem2 =  tradeArray[j][1];
            bitem3 =  tradeArray[j][2];
            
            if (aitem1 == bitem1){
                if (((aitem2 == bitem2) && (aitem3 == bitem3)) || ((aitem2 == bitem3) && (aitem3 == bitem2))) {
                    // Found a duplicate
                    // Now we find the lowest time trade.
                    // The slow trade is copied over (and erased)
                    // The fast trade is moved to i, if not there already.
                    if (atime <= btime){
                        // B is slower
                        ;

                    }
                    else {
                        // A is slower
                        timeArray[i] = timeArray[j];
                        tradeArray[i][0] = tradeArray[j][0];
                        tradeArray[i][1] = tradeArray[j][1];
                        tradeArray[i][2] = tradeArray[j][2];
                        
                        atime  =  timeArray[i];
                        aitem1 =  tradeArray[i][0];
                        aitem2 =  tradeArray[i][1];
                        aitem3 =  tradeArray[i][2];
                    }
                    timeArray[j] = timeArray[K - 1];
                    tradeArray[j][0] = tradeArray[K - 1][0];
                    tradeArray[j][1] = tradeArray[K - 1][1];
                    tradeArray[j][2] = tradeArray[K - 1][2];
                    K = K - 1;
                    // deincrementing J in case the last trade is also a duplicate
                    j = j - 1;
                    
                }
            }
        }
    }
    
    
    // Used by heuristic, getting the lowest trade time
    long long unsigned int minTradeTime = minTradeTimef(timeArray , K );
    long long unsigned int maxTradeTime = maxTradeTimef(timeArray , K );

    // Current State
    int start[I];
    memset( start, 0, I * sizeof(int) );
    // one red paper clip
    start[0] = 1;
    
    
    // Goal State
    int goal[I];
    memset( goal, 0, I * sizeof(int) );
    for (int i = 0 ; i < N; i++){
        int desiredItem = desired[i];
        goal[desiredItem] = goal[desiredItem] + 1;
    }


    
    aStar( goal ,  start ,  minTradeTime , I , tradeArray ,  K , N , timeArray , maxTradeTime);


}


void test_mode(void){
    
}


int main(void){
    if (DEV_MODE){
        test_mode();
    }
    else
        aStarF();
    
}