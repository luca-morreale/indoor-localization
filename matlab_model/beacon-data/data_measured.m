data9 = [
    31 35; ...
    31 40; ...
    31 45; ...
    31 50; ...
    30 55; ...
    29 60; ...
    29 65; ...
    28 70; ...
    27 75; ...
    27 80; ...
    27 85; ...
    25 85; ...
    25 90; ...
    25 95; ...
    23 100; ...
    23 107; ...
    23 110; ...
    22 115; ...
    21 118; ...
    21 120; ...
    21 125; ...
    21 130; ...
    21 133; ...
    20 135; ...
    20 136; ...
    20 139; ...
    20 143; ...
    20 146; ...
    19 148; ...
    19 154; ...
    19 156; ...
    19 162; ...
    18 169; ...
    18 172; ...
    17 177; ...
    17 180; ...
    17 184; ...
    17 188; ...
    16 194; ...
    16 198; ...
    16 200; ... 
    15 204; ...
    15 208; ...
    15 213; ...
    14 220; ...
    14 230; ...
    13 243; ...
    13 245; ...
    12 252; ...
    12 260; ...
    11 270; ...
    11 283; ...
    10 291; ...
    9 300; ...
    8 310; ...
    8 320; ...
    8 336; ...
    7 341; ...
    7 350; ...
    7 363; ...
    6 380; ...
    6 392; ...
    6 400; ...
    6 410; ...
    5 410; ...
    5 420; ...
    5 420; ...
    5 430; ...
    5 440; ...
    4 440; ...
    4 450; ...
    4 450; ...
    4 460; ...
    4 470; ...
    4 470; ...
    4 480; ...
    4 480; ...
    3 490; ...
    3 500; ...
];

%{
data7 = [
    31 35; ...
    31 40; ...
    31 45; ...
    31 50; ...
    30 55; ...
    31 55; ...
    29 60; ...
    29 65; ...
    27 70; ...
    28 70; ...
    28 75; ...
    27 80; ...
    27 80; ...
    26 85; ...
    27 85; ...
    25 90; ...
    25 95; ...
    25 100; ...
    23 105; ...
    23 110; ...
    22 115; ...
    22 120; ...
    21 125; ...
    21 130; ...
    21 135; ...
    20 140; ...
    17 145; ...
    18 145; ...
    19 152; ...
    17 152; ...
    17 155; ...
    18 155; ...
    19 155; ...
    16 160; ...
    17 160; ...
    16 165; ...
    17 165; ...
    16 170; ...
    17 170; ...
    15 175; ...
    17 175; ...
    15 180; ...
    16 180; ... 
    15 185; ...
    15 190; ...    
    16 195; ...
    15 195; ...
    14 200; ...
    15 200; ...
    14 205; ...
    15 205; ...
    13 210; ...
    15 210; ...
    13 215; ...
    15 215; ...
    13 220; ...
    13 225; ...
    13 230; ...
    12 230; ...
    14 230; ...
    12 235; ...
    13 240; ...
    12 240; ...
    12 250; ...
    11 250; ...
    11 260; ...
    11 270; ...
    11 280; ...
    12 280; ...
    10 280; ...
    9 290; ...
    9 300; ...
    8 310; ...
    9 310; ...
    8 320; ...
    8 330; ...
    8 340; ...
    7 350; ...
    7 360; ...
    7 370; ...
    7 380; ...
    6 390; ...
    6 400; ...
    6 410; ...
    6 420; ...
    5 430; ...
    5 440; ...
    4 450; ...
    6 450; ...
    5 460; ...
    4 460; ...
    5 470; ...
    4 470; ...
    0 480; ... % none
    0 490; ... % none
    0 500; ... % none
    ];
%}

data9_side = [
    31 30; ...
    31 35; ...
    31 40; ...
    31 45; ...
    30 50; ...
    30 55; ...
    29 55; ...
    29 60; ...
    29 65; ...
    28 70; ...
    27 75; ...
    27 80; ...
    27 85; ...
    26 85; ...
    25 90; ...
    25 95; ...
    25 100; ...
    23 105; ...
    23 110; ...
    22 115; ...
    22 120; ...
    21 125; ...
    21 130; ...
    20 135; ...
    19 140; ...
    17 150; ...
    17 160; ...
    16 170; ...
    15 180; ...
    15 190; ...
    14 210; ...
    12 230; ...
    12 250; ...
    12 270; ...
    11 300; ...
    9 310; ...
    8 330; ...
    7 350; ...
    7 370; ...
    6 390; ...
    5 410; ...
    5 430; ...
    5 440; ...
    4 450; ...
    4 460; ...
    4 470; ...
    3 480; ...
    3 490; ...
    2 500; ...
    ];
%{
data7_side = [
    31 30; ...
    31 35; ...
    31 40; ...
    31 45; ...
    30 50; ...
    31 50; ...
    30 55; ...
    29 55; ...
    29 60; ...
    29 65; ...
    29 70; ...
    27 70; ...
    28 70; ...
    27 75; ...
    27 80; ...
    26 80; ...
    27 85; ...
    26 85; ...
    25 90; ...
    25 95; ...
    24 95; ...
    25 100; ...
    24 100; ...
    25 105; ...
    23 105; ...
    23 110; ...
    22 115; ...
    22 120; ...
    23 120; ...
    21 125; ...
    21 130; ...
    21 135; ...
    20 135; ...
    19 140; ...
    17 150; ...
    16 160; ...
    16 170; ...
    15 180; ...
    14 190; ...
    13 210; ...
    12 230; ...
    12 250; ...
    13 250; ...
    18 270; ...
    11 290; ...
    8 310; ...
    9 310; ...
    8 330; ...
    7 350; ...
    8 350; ...
    9 350; ...
    6 370; ...
    6 370; ...
    6 390; ...
    6 410; ...
    5 410; ...
    5 430; ...
    5 440; ... % with none
    4 450; ...
    0 470; ... % with none
    0 480; ... % with none
    0 490; ... % none
    ];
%}