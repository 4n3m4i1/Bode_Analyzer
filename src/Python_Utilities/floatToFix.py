def to_fixed(f):
    a = f* (2**15)
    b = int(round(a))
    #if a < 0:
        # next three lines turns b into it's 2's complement.
        #b = abs(b)
        #b = ~b
        #b = b + 1
    return b

h125 = [
 -0.000319382397619666,
-0.00108668649382102,
-0.00229113846060212,
-0.00404294135707087,
-0.00603454299627027,
-0.00745173278232360,
-0.00705325607362336,
-0.00342411744690186,
0.00464761462972035,
0.0177657215755524,
0.0356077612029518,
0.0567965950659227,
0.0790202967917061,
0.0993953755175507,
0.115001843116936,
0.123468590107893,
0.123468590107893,
0.115001843116936,
0.0993953755175507,
0.0790202967917061,
0.0567965950659227,
0.0356077612029518,
0.0177657215755524,
0.00464761462972035,
-0.00342411744690186,
-0.00705325607362336,
-0.00745173278232360,
-0.00603454299627027,
-0.00404294135707087,
-0.00229113846060212,
-0.00108668649382102,
-0.000319382397619666
]
a = []
for i in range(len(h125)):
    a.append(to_fixed(h125[i]))

print(a)