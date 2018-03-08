'''
TEAM 37

Submission for Ultimate Tic Tac Toe Bot Tournament, IIIT-H.
Course: Artificial Intelligence

Vivek Kaushal 20161071
Neha Motlani 20161004
'''

# IMPORTS and CONSTANTS
##################################
import datetime
import copy
import random

BIG = 1e9
##################################

# CLASS
######################################################################################################
class Team37:
'''
Main Class containing all functions.
'''
# INITIALIZATION
##################################
    def __init__(self):
        '''
        Self.dict contains corresponding pairs,
        self.weight assigns a weight to each cell.
        self.var contains:
        count of moves
        limit
        count of limit Reach
        begin and Term Value
        Time Limit
        '''
        self.dict = {'x':1,'o':-1,'-':0,'d':0}
        self.trans = {}
        self.weight = [3,2,2,3,2,3,3,2,2,3,3,2,3,2,2,3]
        self.var = [0,5,0, BIG, BIG, datetime.timedelta(seconds = 14)]
##################################

# MOVE FUNCTION
##################################
    def move(self, board, old_move, flag):
        '''
        Function makes moves and returns 
        the corresponing move.
        '''
        self.trans.clear()
        self.var[3] = datetime.datetime.utcnow()
        self.var[2] = 0
        self.var[0] += 1
        toReturn = board.find_valid_move_cells(old_move)[0]
        for i in xrange(3,100):
            self.trans.clear()
            self.var[1] = i
            valueRet = self.alphaBeta(board, old_move, flag, 1, -BIG, BIG)
            getValue = valueRet[1]
            if(self.var[2] == 0):
                toReturn = getValue
            else:
                break
        return toReturn[0], toReturn[1]
##################################

# HEURISTIC FUNCTION
##################################
    def heuristic(self, board):
        '''
        Heuristic Function, calls for evaluation of board.
        heuristic variable contains:
        Final value to be returned
        Value returned by the block evaluation
        '''
        #final = 0, returned = 1
        heuristicVar = [0,0]
        tempBlock = copy.deepcopy(board.block_status)
        for i in xrange(4):
            for j in xrange(4):
                heuristicVar[1] = self.evaluate(board,i,j,tempBlock)
                heuristicVar[0] += heuristicVar[1]
        heuristicVar[0] += self.blEvaluate(board,tempBlock)*120
        del(tempBlock)
        return heuristicVar[0]
##################################

# MIN MAX FUNCTION
##################################
    def alphaBeta(self, board, old_move, flag, depth, alpha, beta):
        '''
        The AlphaBeta Min Max function.
        Assess output of moves recursively.
        '''
        hashval = [0]
        hashval[0] = hash(str(board.board_status))
        constVal = [alpha,beta]
        if(self.trans.has_key(hashval[0])):
            bounds = self.trans[hashval[0]]
            if(bounds[0] >= constVal[1]):
                return bounds[0],old_move
            if(bounds[1] <= constVal[0]):
                return bounds[1],old_move
            alpha = max(constVal[0],bounds[0])
            beta = min(constVal[1],bounds[1])

        cells = board.find_valid_move_cells(old_move)
        random.shuffle(cells)

        moveStates = ['x','o','d']

        if (flag == moveStates[0]):
            nodeVal = -BIG, cells[0]
            new = moveStates[1]
            tempVar = copy.deepcopy(board.block_status)
            a = alpha
            localMoveCount = [0,0,0]
            temp = [0]
            for chosen in cells :
                if datetime.datetime.utcnow() - self.var[3] >= self.var[5] :
                    self.var[2] = 1
                    break
                board.update(old_move, chosen, flag)
                if (board.find_terminal_state()[0] == moveStates[0]):
                    board.block_status = copy.deepcopy(tempVar)
                    board.board_status[chosen[0]][chosen[1]] = '-'
                    nodeVal = self.var[4],chosen
                    break
                elif (board.find_terminal_state()[0] == moveStates[1]):
                    board.block_status = copy.deepcopy(tempVar)
                    board.board_status[chosen[0]][chosen[1]] = '-'
                    continue
                elif(board.find_terminal_state()[0] == 'NONE'):
                    # x = 0, o = 1, d = 2, tmp1 = 3
                    for i2 in xrange(4):
                        for j2 in xrange(4):
                            if(board.block_status[i2][j2] == moveStates[0]):
                                localMoveCount[0] += 1
                            if(board.block_status[i2][j2] == moveStates[1]):
                                localMoveCount[1] += 1
                            if(board.block_status[i2][j2] == moveStates[2]):
                                localMoveCount[2] += 1
                    if(localMoveCount[0]==localMoveCount[1]):
                        temp[0] = 0
                    elif(localMoveCount[0]>localMoveCount[1]):
                        temp[0] = BIG/2 + 10*(localMoveCount[0]-localMoveCount[1])
                    else:
                        temp[0] = -BIG/2 - 10*(localMoveCount[1]-localMoveCount[0])
                elif( depth >= self.var[1]):
                    temp[0] = self.heuristic(board)
                else:
                    temp[0] = self.alphaBeta(board, chosen, new, depth+1, a, beta)[0]

                board.block_status = copy.deepcopy(tempVar)
                board.board_status[chosen[0]][chosen[1]] = '-'
                
                if(nodeVal[0] < temp[0]):
                    nodeVal = temp[0],chosen
                a = max(a, temp[0])
                if beta <= nodeVal[0] :
                    break
            del(tempVar)

        if (flag == moveStates[1]):
            nodeVal = BIG, cells[0]
            new = moveStates[0]
            tempVar = copy.deepcopy(board.block_status)
            b = beta
            temp = [0]

            for chosen in cells :
                if datetime.datetime.utcnow() - self.var[3] >= self.var[5] :
                    self.var[2] = 1
                    break
                board.update(old_move, chosen, flag)
                if(board.find_terminal_state()[0] == moveStates[1]):
                    board.block_status = copy.deepcopy(tempVar)
                    board.board_status[chosen[0]][chosen[1]] = '-'
                    nodeVal = -1*self.var[4],chosen
                    break
                elif(board.find_terminal_state()[0] == moveStates[0]):
                    board.block_status = copy.deepcopy(tempVar)
                    board.board_status[chosen[0]][chosen[1]] = '-'
                    continue
                elif(board.find_terminal_state()[0] == 'NONE'):
                    # x = 0, o = 1, d = 2
                    localMoveCount[0] = 0
                    localMoveCount[1] = 0
                    localMoveCount[2] = 0
                    for i2 in range(4):
                        for j2 in range(4):
                            if board.block_status[i2][j2] == moveStates[0]:
                                localMoveCount[0] += 1
                            if board.block_status[i2][j2] == moveStates[1]:
                                localMoveCount[1] += 1
                            if board.block_status[i2][j2] == moveStates[2]:
                                localMoveCount[2] += 1
                    if(localMoveCount[0]==localMoveCount[1]):
                        temp[0] = 0
                    elif(localMoveCount[0]>localMoveCount[1]):
                        temp[0] = BIG/2 + 10*(localMoveCount[0]-localMoveCount[1])
                    else:
                        temp[0] = -BIG/2 - 10*(localMoveCount[1]-localMoveCount[0])
                elif(depth >= self.var[1]):
                    temp[0] = self.heuristic(board)
                else:
                    temp[0] = self.alphaBeta(board, chosen, new, depth+1, alpha, b)[0]
                board.block_status = copy.deepcopy(tempVar)
                board.board_status[chosen[0]][chosen[1]] = '-'
                if(nodeVal[0] > temp[0]):
                    nodeVal = temp[0],chosen
                b = min(b, temp[0])
                if alpha >= nodeVal[0] :
                    break
            del(tempVar)

        if(nodeVal[0] <= alpha):
            self.trans[hashval[0]] = [-BIG,nodeVal[0]]
        if(nodeVal[0] > alpha and nodeVal[0] < beta):
            self.trans[hashval[0]] = [nodeVal[0],nodeVal[0]]
        if(nodeVal[0]>=beta):
            self.trans[hashval[0]] = [nodeVal[0],BIG]
        return nodeVal
##################################

# EVALUATION FUNCTION
##################################
    def evaluate(self,board,blx,bly,tempBlock):
        '''
        The evaluation function evaluates 
        choices for moves.
        evaluate variables contain:
        Value 
        Mark
        '''
        evaluateVar = [0,0,0]
        row_col = [[3,3,3,3],[3,3,3,3]]
        for i in xrange(4):
            for j in xrange(4):
                evaluateVar[1] = board.board_status[4*blx+i][4*bly+j]
                dictVal = self.dict[evaluateVar[1]]
                if(dictVal!=0):
                    evaluateVar[0]+=dictVal*self.weight[4*i+j]
                    if (row_col[0][i]==3):
                        row_col[0][i] = dictVal*5
                    elif(dictVal*row_col[0][i]<0):
                        row_col[0][i] = 0
                    row_col[0][i]=row_col[0][i]*16
                    if (row_col[1][j]==3):
                        row_col[1][j] = dictVal*5
                    elif(dictVal*row_col[1][j]<0):
                        row_col[1][j] = 0
                    row_col[1][j]=row_col[1][j]*16

        for i in xrange(4):
                evaluateVar[1] = board.board_status[4*blx+i][4*bly+i]
                dictVal = self.dict[evaluateVar[1]]
                evaluateVar[1] = board.board_status[4*blx+i][4*bly+3-i]
                dictVal = self.dict[evaluateVar[1]]

        draw = 10
        for i in xrange(4):
            if(row_col[1][i]==0):
                draw-=1            
            if(row_col[0][i]==0):
                draw-=1

        if(draw==0):
            tempBlock[blx][bly] = 'd'
            return evaluateVar[2]

        for i in xrange(4):
            if(row_col[0][i]!=3):
                evaluateVar[0]+=row_col[0][i]
            if(row_col[1][i]!=3):
                evaluateVar[0]+=row_col[1][i]

        return evaluateVar[0]
##################################

# BLOCK EVALUATION FUNCTION
##################################
    def blEvaluate(self,board,tempBlock):
        '''
        The Block evaluation function evalues
        choices of moves for a block.
        '''
        blEvaluateVar = [0,0]
        row_col = [[3,3,3,3],[3,3,3,3]]
        for i in xrange(4):
            for j in xrange(4):
                blEvaluateVar[1] = tempBlock[i][j]
                dictVal = self.dict[blEvaluateVar[1]]
                if(blEvaluateVar[1]!='-'):
                    blEvaluateVar[0]+=dictVal*self.weight[4*i+j]
                    if (row_col[0][i]==3):
                        row_col[0][i] = dictVal*5
                    elif(dictVal*row_col[0][i]<=0):
                        row_col[0][i] = 0
                    row_col[0][i]=row_col[0][i]*16
                    if (row_col[1][j]==3):
                        row_col[1][j] = dictVal*5
                    elif(dictVal*row_col[1][j]<=0):
                        row_col[1][j] = 0
                    row_col[1][j]=row_col[1][j]*16

        for i in xrange(4):
                blEvaluateVar[1] = tempBlock[i][i]
                if(blEvaluateVar[1]!='-'):
                    dictVal = self.dict[blEvaluateVar[1]]

                blEvaluateVar[1] = tempBlock[i][3-i]
                if(blEvaluateVar[1]!='-'):
                    dictVal = self.dict[blEvaluateVar[1]]

        for i in xrange(4):
            if(row_col[0][i]!=3):
                blEvaluateVar[0]+=row_col[0][i]
            if(row_col[1][i]!=3):
                blEvaluateVar[0]+=row_col[1][i]

        return blEvaluateVar[0]
##################################

# MTD (f) Search FUNCTION
##################################
    def MTDF(self,board, old_move, flag, depth, f):
        '''
        The MTD Search Funtion.
        '''
        MTDFVar = [f,0]
        bounds = [BIG,-BIG]
        while(bounds[1]<bounds[0]):
            MTDFVar[1] = max(MTDFVar[0],bounds[1]+1)
            tempVar = self.alphaBeta(board,old_move,flag,depth,MTDFVar[1]-1,MTDFVar[1])
            if datetime.datetime.utcnow() - self.var[3] >= self.var[5] :
                self.var[2] = 1
                break
            MTDFVar[0] = tempVar[0]
            if(MTDFVar[0]<MTDFVar[1]):
                bounds[0] = MTDFVar[0]
            else:
                bounds[1] = MTDFVar[0]
        return tempVar
##################################

######################################################################################################

