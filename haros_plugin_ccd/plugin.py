#!/usr/bin/python

#Copyright (c) 2013 Dominik Borowiec, 2016 Andre Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

#=============================================================================
#                         *** CONFIGURATION ***
#=============================================================================
# THIS CONFIGURATION APPLIES FOR C++, BUT IT CAN BE TUNED FOR EACH 
# PROGRAMMING LANGUAGE

# Reserved words and operators for C++
# Note that pure operators can interleave with reserved words 
RESERVED_WORDS = [
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor",
    "bool", "break", "case", "catch", "char", "char16_t", "char32_t", "class",
    "compl", "const", "constexpr", "const_cast", "continue", "decltype",
    "default", "delete", "do", "double", "dynamic_cast", "else", "enum",
    "explicit", "export", "extern", "false", "float", "for", "friend", "goto",
    "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept",
    "not", "not_eq", "nullptr", "operator", "or", "or_eq", "private",
    "protected", "public", "register", "reinterpret_cast", "return", "short",
    "signed", "sizeof", "static", "static_assert", "static_cast", "struct",
    "switch", "template", "this", "thread_local", "throw", "true", "try",
    "typedef", "typeid", "typename", "union", "unsigned", "using", "virtual",
    "void", "volatile", "wchar_t", "while", "xor", "xor_eq"
    ]
    
# Note that the order of the list PURE_OPERATORS is important, because
# because matching is being done from the begining of the list to the end.
PURE_OPERATORS = [
    "::", "++", "--", "(", ")", "[", "]", ".", "->", "++", "--", "+", "-", 
    "!", "~", "*", "&", ".*", "->*", "*", "/", "%", "+", "-", "<<", ">>", 
    "<", "<=", ">", ">=", "==", "!=", "&", "^", "|", "&&", "||", "?:", "=", 
    "+=", "-=", "*=", "/=", "%=", "<<=", ">>=", "&=", "^=", "|=", ",", "\"", 
    "\'", "\\", ";", "{", "}"
    ]

# Comment tokens
ONELINE_COMMENT_TOKEN = "//"
MULTILINE_COMMENT_TOKEN_BEGIN = "/*"
MULTILINE_COMMENT_TOKEN_END = "*/"

# Detector tresholds
# Expected differences (in %) of the metrics computed for snippet with 
# the content of the comment joined with its context should compared with
# the values of metrics for the context only.
# At least given number of tresholds should be applied succesfully to
# state that the comment may contain code. See the code of the script :)

COMMENTED_CODE_MAX_TRESHOLDS = [
    25, #Operators count
    5,  #Distinct operators
    25, #Operands count
    5,  #Distinct operand
    20, #Program length
    5,  #Program vocabulary
    25, #Volume
    20, #Difficulty
    40, #Effort     
]
COMMENTED_CODE_MAX_TRESHOLDS_EXPECTED_PASSES = 7

COMMENTED_CODE_MIN_TRESHOLDS = [
    5, #Operators count
    0, #Distinct operators
    0, #Operands count
    0, #Distinct operand
    0, #Program length
    0, #Program vocabulary
    0, #Volume
    0, #Difficulty
    0, #Effort     
]
COMMENTED_CODE_MIN_TRESHOLDS_EXPECTED_PASSES = 9

#=============================================================================
#                                *** SCRIPT ***
#=============================================================================

import math


class Comment:
    def __init__(self):
        self.content = ""
        self.firstLineNumber = -1
        self.lastLineNumber = -1    
        
        
    def addContent(self, appendedContent, lineNumber):
        self.content = " ".join([self.content, appendedContent])
        self.lastLineNumber = lineNumber
        if self.firstLineNumber == -1:
            self.firstLineNumber = lineNumber
        
        
    def isEmpty(self):
        return self.firstLineNumber == -1
        
        
    def getContent(self):
        return self.content
        
        
    def getFirstLineNumber(self):
        return self.firstLineNumber
        
        
    def getLastLineNumber(self):
        return self.lastLineNumber
        
        
    def getLength(self):
        return self.getLastLineNumber() - self.getFirstLineNumber() + 1
        
        
    def newLine(self):
        self.content += "\n"
        
        
    def __str__(self):
        return "Comment from lines %d-%d:\n%s" %(self.getFirstLineNumber(),
            self.getLastLineNumber(), self.getContent())

            
            
class Halstead:
    def __init__(self, source):
        self.operandsCnt = 0
        self.operatorsCnt = 0
        self.operands = set()
        self.operators = set()
        
        for line in source:
            self._analyzeLine(line)

            
    def _analyzeLine(self, line):
        for lexem in line.split():
            self._analyzeLexem(lexem)

        
    def _analyzeLexem(self, lexem):
        reduct = lexem
        while reduct:
            reduct = self._reduceLexem(reduct)
          
          
    def _reduceLexem(self, lexem):
        nonPureShortestPrefixLen = len(lexem)
        for operator in PURE_OPERATORS:
            if lexem.startswith(operator):
                self.operatorsCnt += 1
                self.operators.add(operator)
                return lexem[len(operator):]
            if operator in lexem:
                nonPureShortestPrefixLen = min(
                    nonPureShortestPrefixLen, 
                    lexem.find(operator)
                )
        nonPureShortestPrefix = lexem[:nonPureShortestPrefixLen]
        for keyword in RESERVED_WORDS:
            if nonPureShortestPrefix == keyword:
                self.operatorsCnt += 1
                self.operators.add(keyword)
                return lexem[nonPureShortestPrefixLen:]
        self.operandsCnt += 1
        self.operands.add(nonPureShortestPrefix)
        return lexem[nonPureShortestPrefixLen:]
    
    
    def getDistinctOperatorsCnt(self):
        return len(self.operators)
    
    
    def getDistinctOperandsCnt(self):
        return len(self.operands)
    
    
    def getTotalOperatorsCnt(self):
        return self.operatorsCnt
    
    
    def getTotalOparandsCnt(self):
        return self.operandsCnt
    
    
    def getLength(self):
        return self.getTotalOperatorsCnt() + self.getTotalOparandsCnt()
    
    
    def getVocabulary(self):
        return self.getDistinctOperatorsCnt() + self.getDistinctOperandsCnt()

        
    def getVolume(self):
        return self.getLength() * math.log(unzero(self.getVocabulary()), 2)
    
    
    def getDifficulty(self):
        return (self.getDistinctOperatorsCnt() / 2 * 
            self.getTotalOparandsCnt() / unzero(
                self.getDistinctOperandsCnt()))
    
    
    def getEffort(self):
        return self.getDifficulty() * self.getVolume()
    
    
    def getValuesVector(self):
        return [
            self.getTotalOperatorsCnt(), 
            self.getDistinctOperatorsCnt(),
            self.getTotalOparandsCnt(),
            self.getDistinctOperandsCnt(),
            self.getLength(),
            self.getVocabulary(),
            self.getVolume(),
            self.getDifficulty(),
            self.getEffort()
        ]
        

        
class CommentFilter:
    def filterComments(self, source):
        """ 
            @input: list of lines of file
            @return: tuple containing list of lines of file and list of 
                Comments
        """
        self.regularLines = []
        self.comments = []
        self.inMultilineComment = False
        self.currentComment = Comment()
        self.lineNumber = 0
        self.blankLines = 0
    
        for line in source:
            if (not line or line.isspace()) and not self.inMultilineComment:
                self.blankLines += 1
            self.currentLine = []        
            while line:
                line = self.reduceLine(line)
            self.regularLines.append(" ".join(self.currentLine))
            self.lineNumber += 1
            if not self.currentComment.isEmpty():
                self.currentComment.newLine()
        
        if not self.currentComment.isEmpty():
            self.comments.append(self.currentComment)
            self.currentComment = Comment()
        
        return (self.regularLines, self.comments)
    
    
    def reduceLine(self, line):
        notInLine = 999999
        multiLineBeginPosition = notInLine
        multiLineEndPosition = notInLine
        oneLinePosition = notInLine
        
        if MULTILINE_COMMENT_TOKEN_BEGIN in line:
            multiLineBeginPosition = line.find(MULTILINE_COMMENT_TOKEN_BEGIN)
            
        if MULTILINE_COMMENT_TOKEN_END in line:
            multiLineEndPosition = line.find(MULTILINE_COMMENT_TOKEN_END)
            
        if ONELINE_COMMENT_TOKEN in line:
            oneLinePosition = line.find(ONELINE_COMMENT_TOKEN)
        
        if (not self.inMultilineComment 
                and oneLinePosition < multiLineBeginPosition):
            self.currentLine.append(line[:oneLinePosition])
            self.currentComment.addContent(
                line[oneLinePosition + len(ONELINE_COMMENT_TOKEN):],
                self.lineNumber    
            )            
        elif self.inMultilineComment and multiLineEndPosition != notInLine:
            self.currentComment.addContent(
                line[:multiLineEndPosition],
                self.lineNumber
            )
            self.inMultilineComment = False
            return (line[multiLineEndPosition + 
                len(MULTILINE_COMMENT_TOKEN_END):])
        elif multiLineBeginPosition != notInLine:
            self.currentLine.append(line[:multiLineBeginPosition])
            self.inMultilineComment = True
            return (line[multiLineBeginPosition + 
                len(MULTILINE_COMMENT_TOKEN_BEGIN):])
        elif self.inMultilineComment:
            self.currentComment.addContent(line, self.lineNumber)
        else:
            if not self.currentComment.isEmpty():
                self.comments.append(self.currentComment)
                self.currentComment = Comment()
            self.currentLine.append(line)
        return ""

    
def analyzeComment(comment, regularLines, args):
    contextLength = max(
        args["minContext"], 
        args["contextMultiplier"] * comment.getLength()
    )
    linesBefore = regularLines[
        max(0, comment.getFirstLineNumber() - contextLength):
        comment.getFirstLineNumber()
    ]
    linesAfter = regularLines[
        comment.getLastLineNumber():
        comment.getLastLineNumber() + contextLength
    ]
    linesCnt = len(linesBefore) + len(linesAfter) + comment.getLength()
    
    codeNoCommentMetrics = Halstead(linesBefore + linesAfter)
    codeWithCommentMetrics =( Halstead(linesBefore + linesAfter + 
        [comment.getContent()]))
    noCommentPerLineValues = [(1.00 * v) / linesCnt 
        for v in codeNoCommentMetrics.getValuesVector()]       
    withCommentPerLineValues = [(1.00 * v) / linesCnt 
        for v in codeWithCommentMetrics.getValuesVector()]

    diffValues = [abs(w - n) / unzero(n) * 100 for 
        (n, w) in zip(noCommentPerLineValues, withCommentPerLineValues)
    ]
        
    maxTresholdHits = [d <= t for (d, t) in 
        zip (diffValues, COMMENTED_CODE_MAX_TRESHOLDS)]
    totalMaxTresholdHit = (sum(x > 0 for x in maxTresholdHits) >= 
        COMMENTED_CODE_MAX_TRESHOLDS_EXPECTED_PASSES)
    minTresholdHits = [d >= t for (d, t) in 
        zip (diffValues, COMMENTED_CODE_MIN_TRESHOLDS)]
    totalMinTresholdHit = (sum(x > 0 for x in minTresholdHits) >= 
        COMMENTED_CODE_MIN_TRESHOLDS_EXPECTED_PASSES)       
    hit = totalMaxTresholdHit and totalMinTresholdHit
        
    if (args["verbose"] >= 1 and hit) or (args["verbose"] == 3):
        print comment
        if (args["verbose"] >= 2):
            print "Analyzed context lines count: %s." % contextLength
            Halstead.printStatistics([
                codeNoCommentMetrics.getValuesVector(),
                codeWithCommentMetrics.getValuesVector(),
                noCommentPerLineValues,
                withCommentPerLineValues,
                diffValues,
                maxTresholdHits
            ], ["-Cmt", "+Cmt", "-Cmt/l", "+Cmt/l", "diff%", "<maxT?"])
    if hit:
        print ("Lines %s-%s seems to be commented code." 
            %(comment.getFirstLineNumber(), comment.getLastLineNumber()))
    if (args["verbose"] >= 1 and hit) or (args["verbose"] == 3):
        print ""
        print "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"

            
def unzero(v):
    if v == 0:
        return 0.00000001
    else:
        return v


def file_analysis(iface, scope):
    with open(scope.path, "r") as f:
        code = f.read().splitlines()
    com_filter = CommentFilter()
    (regularLines, comments) = com_filter.filterComments(code)
    sloc = scope.lines - com_filter.blankLines
    iface.report_metric("sloc", sloc)
    ncomments = sum(map(Comment.getLength, comments))
    iface.report_metric("comments", ncomments)
    iface.report_metric("comment_ratio", float(ncomments) / sloc)
    metrics = Halstead(regularLines)
    volume = metrics.getVolume()
    iface.report_metric("halstead_volume", volume)
    if volume > 8000:
        iface.report_violation("halstead_volume_above_8000",
                               "Halstead volume of " + str(volume))
    iface.report_metric("halstead_time", metrics.getEffort() / 18.0)
    bugs = metrics.getVolume() / 3000.0
    iface.report_metric("halstead_bugs", bugs)
    if bugs > 2:
        iface.report_violation("halstead_bugs_above_2",
                                "Halstead Bugs of " + str(bugs))
