# Complete failed-case IK trace

Units: first three joint entries mm, last three rad. All targets are reachable by construction.
Each branch includes its initial guess, pre/post-wrap result, limits and termination reason.
Full iteration/Jacobian/damping histories are in `traces.json.gz`.

## joint2_upper (limits)

Target: `[300. , 300. , 300. ,   0.3,   0.5,  -0.4]`

Caller seed: `[300.1 , 299.9 , 300.1 ,   0.35,   0.47,  -0.36]`

Selected branch: `None`; strict pass: `False`.

Classification: **numerical_boundary_rejection**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | no candidate | — | — | no accepted in-limit candidate |
| iterations_200 | no candidate | — | — | no accepted in-limit candidate |
| iterations_200_tight | `[300. , 300. , 300. ,   0.3,   0.5,  -0.4]` | 1.271057486e-13 | 6.689255819e-16 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.e+02,  3.e+02,  3.e+02,  3.e-01,  5.e-01, -4.e-01]` | 0 | 1.350520526e-16 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [ 2.983648338895e+02,  3.060646052997e+02,  3.020523497238e+02,
  2.896524623924e-01,  4.959639269368e-01, -4.040226452690e-01]
before wrap : [300.000000001113, 300.000000002393, 300.00000000053 ,   0.300000000011,
   0.5           ,  -0.400000000032]
final joints: [300.000000001113, 300.000000002393, 300.00000000053 ,   0.300000000011,
   0.5           ,  -0.400000000032]
limit excess: [0.000000000000e+00, 2.393221620878e-09, 0.000000000000e+00,
 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: False.

Before wrap: 7.70055258812e-12 mm / 2.86136147785e-11 rad. After wrap: **7.70055258812e-12 mm / 2.86135403625e-11 rad**.

Max balanced Jacobian condition: 2.23036; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.364833889477, 306.06460529973 , 302.052349723799,  -2.851940191197,
   2.645628726653,   2.737570008321]
before wrap : [300.000000003723, 300.000000007602, 300.000000001968,  -2.901237219078,
   2.62972862893 ,   2.825957343621]
final joints: [300.000000003723, 300.000000007602, 300.000000001968,  -2.901237219078,
   2.62972862893 ,   2.825957343621]
limit excess: [0.000000000000e+00, 7.602352525282e-09, 0.000000000000e+00,
 5.450427288858e-01, 2.735341387381e-01, 0.000000000000e+00]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: False.

Before wrap: 3.04911828963e-11 mm / 9.2924314343e-11 rad. After wrap: **3.0526980005e-11 mm / 9.29242008745e-11 rad**.

Max balanced Jacobian condition: 2.19162; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## joint6_upper (limits)

Target: `[300.           ,   0.           , 300.           ,   0.3          ,
   0.5          ,   3.14159265359]`

Caller seed: `[ 3.00100000000e+02, -1.00000000000e-01,  3.00100000000e+02,
  3.50000000000e-01,  4.70000000000e-01,  3.14159265359e+00]`

Selected branch: `0`; strict pass: `False`.

Classification: **nonperiodic_wrist_wrapping**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[3.0000000000e+02, 3.6794583463e-09, 3.0000000000e+02, 2.9999999999e-01, 5.0000000001e-01, 3.1413958656e+00]` | 0.01968874005 | 0.0001967781371 | raw_residual_tolerance; in limits: True |
| iterations_200 | `[3.0000000000e+02, 3.6794583463e-09, 3.0000000000e+02, 2.9999999999e-01, 5.0000000001e-01, 3.1413958656e+00]` | 0.01968874005 | 0.0001967781371 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[3.0000000000e+02, 9.8827773346e-14, 3.0000000000e+02, 3.0000000000e-01, 5.0000000000e-01, 3.1413958656e+00]` | 0.01968874005 | 0.0001967781729 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -3.5453097005e-11,  3.0000000000e+02,  3.0000000000e-01,  5.0000000000e-01,  3.1415926536e+00]` | 3.356865288e-11 | 1.840023641e-13 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [ 2.986412520556e+02,  6.314292719778e+00,  2.992562490516e+02,
  2.659096306722e-01,  4.953022929943e-01, -3.134440732759e+00]
before wrap : [ 2.999999999996e+02,  3.679458346296e-09,  3.000000000002e+02,
  2.999999999861e-01,  5.000000000092e-01, -3.141789441560e+00]
final joints: [2.999999999996e+02, 3.679458346296e-09, 3.000000000002e+02,
 2.999999999861e-01, 5.000000000092e-01, 3.141395865620e+00]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: True.

Before wrap: 1.0019650308e-11 mm / 3.90278805296e-11 rad. After wrap: **0.0196887400504 mm / 0.000196778137051 rad**.

Max balanced Jacobian condition: 2.13621; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [ 2.986412520556e+02,  6.314292719778e+00,  2.992562490516e+02,
 -2.875683022918e+00,  2.646290360596e+00,  7.151920830776e-03]
before wrap : [ 2.999999999998e+02, -1.557983813053e-09,  2.999999999992e+02,
 -2.901237219111e+00,  2.629728628930e+00,  8.411530670607e-02]
final joints: [ 2.999999999998e+02, -1.557983813053e-09,  2.999999999992e+02,
 -2.901237219111e+00,  2.629728628930e+00,  8.411530670607e-02]
limit excess: [0.            , 0.            , 0.            , 0.545042728919,
 0.273534138737, 0.            ]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: False.

Before wrap: 6.3089953373e-12 mm / 1.90449749035e-11 rad. After wrap: **6.30784288659e-12 mm / 1.90451796183e-11 rad**.

Max balanced Jacobian condition: 2.13794; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## joint6_upper_interior (near_limits)

Target: `[300.           ,   0.           , 300.           ,   0.3          ,
   0.5          ,   3.14158265359]`

Caller seed: `[ 3.00100000000e+02, -1.00000000000e-01,  3.00100000000e+02,
  3.50000000000e-01,  4.70000000000e-01,  3.14159265359e+00]`

Selected branch: `0`; strict pass: `False`.

Classification: **nonperiodic_wrist_wrapping**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[3.0000000000e+02, 3.6794137496e-09, 3.0000000000e+02, 2.9999999999e-01, 5.0000000001e-01, 3.1413858660e+00]` | 0.01968870267 | 0.0001967777635 | raw_residual_tolerance; in limits: True |
| iterations_200 | `[3.0000000000e+02, 3.6794137496e-09, 3.0000000000e+02, 2.9999999999e-01, 5.0000000001e-01, 3.1413858660e+00]` | 0.01968870267 | 0.0001967777635 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[3.000000000e+02, 8.706896269e-14, 3.000000000e+02, 3.000000000e-01, 5.000000000e-01, 3.141385866e+00]` | 0.01968870267 | 0.0001967777993 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -3.5215333894e-11,  3.0000000000e+02,  3.0000000000e-01,  5.0000000000e-01,  3.1415826536e+00]` | 3.407878222e-11 | 1.842958845e-13 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [ 2.986412528415e+02,  6.314293374422e+00,  2.992562487941e+02,
  2.659096021967e-01,  4.953023966054e-01, -3.134450718156e+00]
before wrap : [ 2.999999999996e+02,  3.679413749581e-09,  3.000000000002e+02,
  2.999999999861e-01,  5.000000000092e-01, -3.141799441186e+00]
final joints: [2.999999999996e+02, 3.679413749581e-09, 3.000000000002e+02,
 2.999999999861e-01, 5.000000000092e-01, 3.141385865994e+00]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: True.

Before wrap: 1.00114740247e-11 mm / 3.90277444878e-11 rad. After wrap: **0.0196887026698 mm / 0.000196777763453 rad**.

Max balanced Jacobian condition: 2.13621; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [ 2.986412528415e+02,  6.314293374422e+00,  2.992562487941e+02,
 -2.875683051393e+00,  2.646290256984e+00,  7.141935433304e-03]
before wrap : [ 2.999999999998e+02, -1.558007397191e-09,  2.999999999992e+02,
 -2.901237219111e+00,  2.629728628930e+00,  8.410530688788e-02]
final joints: [ 2.999999999998e+02, -1.558007397191e-09,  2.999999999992e+02,
 -2.901237219111e+00,  2.629728628930e+00,  8.410530688788e-02]
limit excess: [0.            , 0.            , 0.            , 0.545042728919,
 0.273534138737, 0.            ]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: False.

Before wrap: 5.98801643699e-12 mm / 1.90450331246e-11 rad. After wrap: **5.98673473371e-12 mm / 1.90451936013e-11 rad**.

Max balanced Jacobian condition: 2.13794; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## all_lower (limits)

Target: `[   0.            , -300.            ,    0.            ,
   -2.356194490192,   -2.356194490192,   -3.14159265359 ]`

Caller seed: `[ 1.000000000000e-01, -3.000000000000e+02,  1.000000000000e-01,
 -2.306194490192e+00, -2.356194490192e+00, -3.101592653590e+00]`

Selected branch: `1`; strict pass: `False`.

Classification: **nonperiodic_wrist_wrapping**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 1.4727273704e-13, -3.0000000000e+02,  1.1320703542e-13, -2.3561944902e+00, -2.3561944902e+00, -3.1413958582e+00]` | 0.01969021134 | 0.0001967928776 | raw_residual_tolerance; in limits: True |
| iterations_200 | `[ 1.4727273704e-13, -3.0000000000e+02,  1.1320703542e-13, -2.3561944902e+00, -2.3561944902e+00, -3.1413958582e+00]` | 0.01969021134 | 0.0001967928776 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 1.4727273704e-13, -3.0000000000e+02,  1.1320703542e-13, -2.3561944902e+00, -2.3561944902e+00, -3.1413958582e+00]` | 0.01969021134 | 0.0001967928776 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 2.0079923438e-11, -3.0000000000e+02,  7.8593761392e-12, -2.3561944902e+00, -2.3561944902e+00, -3.1415926536e+00]` | 2.009012812e-11 | 3.141333593e-14 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [ 9.219156007549e-01, -2.996011285021e+02, -3.328969528422e+00,
  7.802526649778e-01, -8.007657523451e-01, -5.987651632681e-02]
before wrap : [ 2.014874520501e-10, -3.000000000025e+02, -3.231814742249e-09,
  7.519314190032e-01, -7.971499078219e-01, -6.830778182113e-02]
final joints: [ 2.014874520501e-10, -3.000000000025e+02, -3.231814742249e-09,
  7.519314190032e-01, -7.971499078219e-01, -6.830778182113e-02]
limit excess: [0.000000000000e+00, 2.456999936840e-09, 3.231814742249e-09,
 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00]
```

Termination: raw_residual_tolerance; iterations: 5; final in limits: False.

Before wrap: 1.61129035313e-11 mm / 5.73952268166e-11 rad. After wrap: **1.61236284954e-11 mm / 5.73952790435e-11 rad**.

Max balanced Jacobian condition: 2.8723; damping range 1e-09...0.0001; rejected steps: 0; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [   0.921915600755, -299.601128502133,   -3.328969528422,
   -2.361339988612,   -2.340826901245,    3.081716137263]
before wrap : [ 1.472727370409e-13, -3.000000000000e+02,  1.132070354181e-13,
 -2.356194490192e+00, -2.356194490192e+00,  3.141789448955e+00]
final joints: [ 1.472727370409e-13, -3.000000000000e+02,  1.132070354181e-13,
 -2.356194490192e+00, -2.356194490192e+00, -3.141395858225e+00]
limit excess: [0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00,
 3.996802888651e-15, 0.000000000000e+00, 0.000000000000e+00]
```

Termination: raw_residual_tolerance; iterations: 6; final in limits: True.

Before wrap: 1.7288250917e-13 mm / 3.01393793577e-15 rad. After wrap: **0.0196902113418 mm / 0.000196792877609 rad**.

Max balanced Jacobian condition: 2.90254; damping range 1e-10...0.0001; rejected steps: 0; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_-0.01 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.586676209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.616676209396e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9982964251e+02, -1.2509650378e-03,  2.9999620779e+02,  4.7668394467e-01, -1.5910067719e+00, -6.2322213033e-01]` | 0.0003986123079 | 0.002733765172 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02,  1.1754766743e-11,  3.0000000000e+02,  6.9999999997e-01, -1.5866762094e+00, -4.0000000003e-01]` | 1.301912856e-11 | 2.591953262e-13 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -3.4525863359e-14,  3.0000000000e+02,  7.0000000000e-01, -1.5866762094e+00, -4.0000000000e-01]` | 2.842170943e-14 | 1.21127395e-16 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -2.5827699670e-11,  3.0000000000e+02,  7.0000000007e-01, -1.5866762094e+00, -3.9999999993e-01]` | 2.86210142e-11 | 7.06866823e-13 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.754797949328,   5.972044653539, 299.775399324232,   3.012743247793,
  -1.536570376155,   1.881925791503]
before wrap : [ 3.007602103558e+02, -2.635788083819e-03,  3.000073573278e+02,
  2.165590680526e+00, -1.556984466270e+00,  1.064946210400e+00]
final joints: [ 3.007602103558e+02, -2.635788083819e-03,  3.000073573278e+02,
  2.165590680526e+00, -1.556984466270e+00,  1.064946210400e+00]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 4.53916078148e-05 mm / 0.00878665568272 rad. After wrap: **4.53916078148e-05 mm / 0.00878665568272 rad**.

Max balanced Jacobian condition: 140.226; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [ 2.997547979493e+02,  5.972044653539e+00,  2.997753993242e+02,
 -1.288494057965e-01, -1.605022277435e+00, -1.259666862087e+00]
before wrap : [ 2.998296425110e+02, -1.250965037797e-03,  2.999962077875e+02,
  4.766839446730e-01, -1.591006771889e+00, -6.232221303272e-01]
final joints: [ 2.998296425110e+02, -1.250965037797e-03,  2.999962077875e+02,
  4.766839446730e-01, -1.591006771889e+00, -6.232221303272e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000398612307877 mm / 0.00273376517187 rad. After wrap: **0.000398612307877 mm / 0.00273376517187 rad**.

Max balanced Jacobian condition: 157.837; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_-0.001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.577676209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.607676209396e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9985299585e+02, -2.0578585872e-03,  2.9999748220e+02,  3.0603555878e-01, -1.5856172216e+00, -7.9379240442e-01]` | 7.450294495e-05 | 0.00198666893 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02,  1.7627807014e-11,  3.0000000000e+02,  6.9999999531e-01, -1.5776762095e+00, -4.0000000469e-01]` | 2.367582986e-11 | 4.685730453e-12 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02,  3.3087383328e-13,  3.0000000000e+02,  6.9999999991e-01, -1.5776762094e+00, -4.0000000009e-01]` | 4.865013002e-13 | 9.204699663e-14 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  2.2901805821e-11,  3.0000000000e+02,  6.9999999999e-01, -1.5776762094e+00, -4.0000000001e-01]` | 2.521165834e-11 | 6.885648955e-15 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.755025754053,   5.976221646493, 299.770465449828,   2.78091005687 ,
  -1.541865977264,   1.650208801505]
before wrap : [ 3.007803802729e+02,  1.335895285463e-04,  2.999983486812e+02,
  1.856718014471e+00, -1.558038326572e+00,  7.562899917102e-01]
final joints: [ 3.007803802729e+02,  1.335895285463e-04,  2.999983486812e+02,
  1.856718014471e+00, -1.558038326572e+00,  7.562899917102e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 6.81829155946e-05 mm / 0.0115923574095 rad. After wrap: **6.81829155946e-05 mm / 0.0115923574095 rad**.

Max balanced Jacobian condition: 135.659; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.755025754053,   5.976221646493, 299.770465449828,  -0.36068259672 ,
  -1.599726676326,  -1.491383852085]
before wrap : [ 2.998529958533e+02, -2.057858587189e-03,  2.999974821978e+02,
  3.060355587846e-01, -1.585617221554e+00, -7.937924044225e-01]
final joints: [ 2.998529958533e+02, -2.057858587189e-03,  2.999974821978e+02,
  3.060355587846e-01, -1.585617221554e+00, -7.937924044225e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.45029449524e-05 mm / 0.00198666892959 rad. After wrap: **7.45029449509e-05 mm / 0.00198666892959 rad**.

Max balanced Jacobian condition: 254.285; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_-0.0001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.576776209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.606776209396e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9985943450e+02, -2.0402762287e-03,  2.9999770358e+02,  2.8016179688e-01, -1.5852854583e+00, -8.1965388352e-01]` | 6.072281538e-05 | 0.001857976797 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -8.5806944562e-11,  3.0000000000e+02,  7.0955429848e-01, -1.5765761959e+00, -3.9045003290e-01]` | 3.237253914e-11 | 6.430724058e-11 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -3.3480880209e-12,  3.0000000000e+02,  7.0955368034e-01, -1.5765762089e+00, -3.9045065075e-01]` | 1.223454731e-12 | 2.520290048e-12 | iteration_limit; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -1.0679813727e-11,  3.0000000000e+02,  7.0955365516e-01, -1.5765762094e+00, -3.9045067593e-01]` | 1.17696423e-11 | 3.049524861e-15 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.75504533529 ,   5.976639352419, 299.7699719279  ,   2.753318529815,
  -1.542293189815,   1.62262889406 ]
before wrap : [ 3.007575680357e+02,  2.782732274490e-04,  2.999969919815e+02,
  1.820395511250e+00, -1.558001233634e+00,  7.199861681465e-01]
final joints: [ 3.007575680357e+02,  2.782732274490e-04,  2.999969919815e+02,
  1.820395511250e+00, -1.558001233634e+00,  7.199861681465e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.23644202518e-05 mm / 0.0117274172938 rad. After wrap: **7.23644202523e-05 mm / 0.0117274172938 rad**.

Max balanced Jacobian condition: 130.382; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.75504533529 ,   5.976639352419, 299.7699719279  ,  -0.388274123775,
  -1.599299463775,  -1.51896375953 ]
before wrap : [ 2.998594345000e+02, -2.040276228722e-03,  2.999977035845e+02,
  2.801617968765e-01, -1.585285458296e+00, -8.196538835162e-01]
final joints: [ 2.998594345000e+02, -2.040276228722e-03,  2.999977035845e+02,
  2.801617968765e-01, -1.585285458296e+00, -8.196538835162e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 6.07228153785e-05 mm / 0.00185797679718 rad. After wrap: **6.07228153785e-05 mm / 0.00185797679718 rad**.

Max balanced Jacobian condition: 264.365; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_0.0 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.576676209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.606676209396e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9986428318e+02, -1.9818266927e-03,  2.9999777871e+02,  2.8242311258e-01, -1.5851520986e+00, -8.1739329592e-01]` | 5.509564994e-05 | 0.001797987879 | iteration_limit; in limits: True |
| iterations_200 | `[ 2.9999999847e+02, -3.9511680294e-08,  2.9999999995e+02,  7.0187876254e-01, -1.5766368790e+00, -3.9812208917e-01]` | 7.959898808e-09 | 3.69278128e-08 | iteration_limit; in limits: True |
| iterations_200_tight | `[ 2.9999999847e+02, -3.9511680294e-08,  2.9999999995e+02,  7.0187876254e-01, -1.5766368790e+00, -3.9812208917e-01]` | 7.959898808e-09 | 3.69278128e-08 | iteration_limit; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -9.0712490014e-11,  3.0000000000e+02,  7.0002174824e-01, -1.5766757541e+00, -3.9997826162e-01]` | 9.450286768e-11 | 4.94833275e-12 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.755047475073,   5.976685764186, 299.769917090712,   2.750202217367,
  -1.542339296953,   1.619513872675]
before wrap : [ 3.007547232704e+02,  2.906311168278e-04,  2.999968381536e+02,
  1.816293527769e+00, -1.557995564347e+00,  7.158862280418e-01]
final joints: [ 3.007547232704e+02,  2.906311168278e-04,  2.999968381536e+02,
  1.816293527769e+00, -1.557995564347e+00,  7.158862280418e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.2859267845e-05 mm / 0.0117402141489 rad. After wrap: **7.28592678448e-05 mm / 0.0117402141489 rad**.

Max balanced Jacobian condition: 129.766; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.755047475073,   5.976685764186, 299.769917090712,  -0.391390436223,
  -1.599253356637,  -1.522078780915]
before wrap : [ 2.998642831759e+02, -1.981826692689e-03,  2.999977787091e+02,
  2.824231125831e-01, -1.585152098581e+00, -8.173932959202e-01]
final joints: [ 2.998642831759e+02, -1.981826692689e-03,  2.999977787091e+02,
  2.824231125831e-01, -1.585152098581e+00, -8.173932959202e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 5.50956499366e-05 mm / 0.00179798787902 rad. After wrap: **5.50956499374e-05 mm / 0.00179798787902 rad**.

Max balanced Jacobian condition: 268.661; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_0.0001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.576576209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.606576209396e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9986382520e+02, -1.9946160603e-03,  2.9999778889e+02,  2.7776057248e-01, -1.5851500303e+00, -8.2205371728e-01]` | 5.506151299e-05 | 0.001796799905 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -8.6646123343e-11,  3.0000000000e+02,  7.0000064266e-01, -1.5765761959e+00, -3.9999935763e-01]` | 3.23453164e-11 | 6.423790081e-11 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -5.0989318306e-12,  3.0000000000e+02,  7.0000003775e-01, -1.5765762086e+00, -3.9999996226e-01]` | 1.946835231e-12 | 3.773481903e-12 | iteration_limit; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -2.7123764612e-11,  3.0000000000e+02,  7.0000000008e-01, -1.5765762094e+00, -3.9999999992e-01]` | 2.978703626e-11 | 8.276657282e-15 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.755049607674,   5.97673217595 , 299.769862253242,   2.747075823435,
  -1.54238512699 ,   1.616388769805]
before wrap : [ 3.007518168291e+02,  3.021945049422e-04,  2.999966837992e+02,
  1.812178410667e+00, -1.557989587813e+00,  7.117731479243e-01]
final joints: [ 3.007518168291e+02,  3.021945049422e-04,  2.999966837992e+02,
  1.812178410667e+00, -1.557989587813e+00,  7.117731479243e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.33530661926e-05 mm / 0.0117525582495 rad. After wrap: **7.33530661914e-05 mm / 0.0117525582495 rad**.

Max balanced Jacobian condition: 129.145; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.755049607674,   5.97673217595 , 299.769862253242,  -0.394516830155,
  -1.5992075266  ,  -1.525203883784]
before wrap : [ 2.998638252045e+02, -1.994616060326e-03,  2.999977888928e+02,
  2.777605724764e-01, -1.585150030265e+00, -8.220537172805e-01]
final joints: [ 2.998638252045e+02, -1.994616060326e-03,  2.999977888928e+02,
  2.777605724764e-01, -1.585150030265e+00, -8.220537172805e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 5.50615129928e-05 mm / 0.0017967999053 rad. After wrap: **5.50615129939e-05 mm / 0.0017967999053 rad**.

Max balanced Jacobian condition: 268.729; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_0.001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.575676209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.605676209396e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9987486058e+02, -1.8959374320e-03,  2.9999806351e+02,  2.5355888792e-01, -1.5848019728e+00, -8.4624348047e-01]` | 4.145828849e-05 | 0.00161928095 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -3.4944522946e-11,  3.0000000000e+02,  7.0000000631e-01, -1.5756762093e+00, -3.9999999369e-01]` | 3.189433781e-11 | 6.309107544e-12 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -6.9292728711e-13,  3.0000000000e+02,  7.0000000012e-01, -1.5756762094e+00, -3.9999999988e-01]` | 6.827128885e-13 | 1.239087221e-13 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -3.5713689372e-14,  3.0000000000e+02,  7.0000000000e-01, -1.5756762094e+00, -4.0000000000e-01]` | 2.308990589e-13 | 2.334290763e-17 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.755068477885,   5.977149881649, 299.769368703436,   2.718486940729,
  -1.542784906083,   1.587811506625]
before wrap : [ 3.007229114452e+02,  3.682375906846e-04,  2.999952748351e+02,
  1.774558351675e+00, -1.557922092233e+00,  6.741711370810e-01]
final joints: [ 3.007229114452e+02,  3.682375906846e-04,  2.999952748351e+02,
  1.774558351675e+00, -1.557922092233e+00,  6.741711370810e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.80182055536e-05 mm / 0.0118428309351 rad. After wrap: **7.80182055536e-05 mm / 0.0118428309351 rad**.

Max balanced Jacobian condition: 128.337; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.755068477885,   5.977149881649, 299.769368703436,  -0.42310571286 ,
  -1.598807747506,  -1.553781146965]
before wrap : [ 2.998748605819e+02, -1.895937432008e-03,  2.999980635114e+02,
  2.535588879201e-01, -1.584801972812e+00, -8.462434804728e-01]
final joints: [ 2.998748605819e+02, -1.895937432008e-03,  2.999980635114e+02,
  2.535588879201e-01, -1.584801972812e+00, -8.462434804728e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 4.1458288486e-05 mm / 0.00161928095036 rad. After wrap: **4.1458288486e-05 mm / 0.00161928095036 rad**.

Max balanced Jacobian condition: 280.667; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_-1_offset_0.01 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.566676209396,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.596676209396e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 3.0025246237e+02, -3.3949236647e-03,  2.9998403499e+02,  1.3605151856e+00, -1.5560502191e+00,  2.6028342550e-01]` | 0.0001176992733 | 0.01053454618 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -3.1284721195e-14,  3.0000000000e+02,  7.0000000000e-01, -1.5666762094e+00, -4.0000000000e-01]` | 4.019436694e-14 | 5.905056595e-16 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -3.1284721195e-14,  3.0000000000e+02,  7.0000000000e-01, -1.5666762094e+00, -4.0000000000e-01]` | 4.019436694e-14 | 5.905056595e-16 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -3.7946955171e-14,  3.0000000000e+02,  7.0000000000e-01, -1.5666762094e+00, -4.0000000000e-01]` | 2.291431471e-13 | 1.921011209e-16 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.75522518061 ,   5.981326851925, 299.764432041286,   2.393830028208,
  -1.545337313935,   1.263270786652]
before wrap : [ 3.002524623737e+02, -3.394923664670e-03,  2.999840349902e+02,
  1.360515185629e+00, -1.556050219059e+00,  2.602834255020e-01]
final joints: [ 3.002524623737e+02, -3.394923664670e-03,  2.999840349902e+02,
  1.360515185629e+00, -1.556050219059e+00,  2.602834255020e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000117699273282 mm / 0.0105345461847 rad. After wrap: **0.000117699273282 mm / 0.0105345461847 rad**.

Max balanced Jacobian condition: 121.018; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.75522518061 ,   5.981326851925, 299.764432041286,  -0.747762625382,
  -1.596255339654,  -1.878321866938]
before wrap : [ 3.000000000000e+02,  6.177492228368e-12,  3.000000000000e+02,
 -1.913232299213e-01, -1.586676208076e+00, -1.290891110827e+00]
final joints: [ 3.000000000000e+02,  6.177492228368e-12,  3.000000000000e+02,
 -1.913232299213e-01, -1.586676208076e+00, -1.290891110827e+00]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: raw_residual_tolerance; iterations: 41; final in limits: True.

Before wrap: 7.10639363002e-12 mm / 1.41281943188e-13 rad. After wrap: **7.04955800261e-12 mm / 1.41284104753e-13 rad**.

Max balanced Jacobian condition: 231.64; damping range 1e-12...0.0001; rejected steps: 32; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## nominal_pole_-1 (nominal_poles)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
  -1.570796326795,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01, -1.600796326795e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9995667408e+02, -7.3936630607e-04,  2.9999956834e+02,  7.7645111642e-02, -1.5840912137e+00, -1.0220672861e+00]` | 0.0004058638921 | 0.0004971226088 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -6.3149689701e-12,  3.0000000000e+02,  7.0000000004e-01, -1.5707963268e+00, -3.9999999996e-01]` | 6.594816574e-12 | 2.222236753e-13 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -2.0921155461e-14,  3.0000000000e+02,  7.0000000000e-01, -1.5707963268e+00, -4.0000000000e-01]` | 2.842170943e-14 | 1.980968085e-16 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02, -1.3916572955e-12,  3.0000000000e+02,  7.0000000000e-01, -1.5707963268e+00, -4.0000000000e-01]` | 1.47792889e-12 | 6.169781423e-15 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [299.755160664655,   5.979414701825, 299.766692251966,   2.550124826881,
  -1.5445182559  ,   1.419512393928]
before wrap : [ 3.004922769592e+02, -7.212443133731e-04,  2.999878810428e+02,
  1.554965656373e+00, -1.557153609863e+00,  4.546679183219e-01]
final joints: [ 3.004922769592e+02, -7.212443133731e-04,  2.999878810428e+02,
  1.554965656373e+00, -1.557153609863e+00,  4.546679183219e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000105671471012 mm / 0.0116292745776 rad. After wrap: **0.000105671471013 mm / 0.0116292745776 rad**.

Max balanced Jacobian condition: 127.829; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [299.755160664655,   5.979414701825, 299.766692251966,  -0.591467826709,
  -1.597074397689,  -1.722080259661]
before wrap : [ 2.999566740820e+02, -7.393663060652e-04,  2.999995683396e+02,
  7.764511164212e-02, -1.584091213692e+00, -1.022067286141e+00]
final joints: [ 2.999566740820e+02, -7.393663060652e-04,  2.999995683396e+02,
  7.764511164212e-02, -1.584091213692e+00, -1.022067286141e+00]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000405863892056 mm / 0.00049712260875 rad. After wrap: **0.000405863892057 mm / 0.00049712260875 rad**.

Max balanced Jacobian condition: 296.353; damping range 1e-09...0.0001; rejected steps: 45; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_1_offset_-0.001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.563856830421,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.533856830421e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9999830187e+02, -5.2346971395e-06,  2.9999996954e+02,  6.7203616515e-01,  1.5626727711e+00, -3.7204882887e-01]` | 1.978512643e-05 | 4.450252516e-05 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -5.5685161606e-13,  3.0000000000e+02,  6.9999999712e-01,  1.5638568303e+00, -3.9999999712e-01]` | 2.859879345e-13 | 2.877542776e-12 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -1.2388451761e-14,  3.0000000000e+02,  6.9999999994e-01,  1.5638568304e+00, -3.9999999994e-01]` | 1.145715735e-13 | 5.6580177e-14 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  1.2272407902e-11,  3.0000000000e+02,  7.0000001766e-01,  1.5638568312e+00, -4.0000001765e-01]` | 4.320847708e-11 | 1.764678778e-11 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.318767708057,   5.976703989423, 299.713040163277,  -0.369723616316,
   1.541387717831,   0.642609233168]
before wrap : [ 2.999983018725e+02, -5.234697139470e-06,  2.999999695373e+02,
  6.720361651496e-01,  1.562672771097e+00, -3.720488288746e-01]
final joints: [ 2.999983018725e+02, -5.234697139470e-06,  2.999999695373e+02,
  6.720361651496e-01,  1.562672771097e+00, -3.720488288746e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 1.97851264302e-05 mm / 4.45025251553e-05 rad. After wrap: **1.97851264304e-05 mm / 4.45025251553e-05 rad**.

Max balanced Jacobian condition: 962.267; damping range 1e-10...0.0001; rejected steps: 44; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.318767708057,   5.976703989423, 299.713040163277,   2.771869037274,
   1.600204935759,  -2.498983420421]
before wrap : [ 2.994236969585e+02, -2.572762910317e-03,  2.999877159594e+02,
  1.294671074098e+00,  1.587715151679e+00, -9.944132184372e-01]
final joints: [ 2.994236969585e+02, -2.572762910317e-03,  2.999877159594e+02,
  1.294671074098e+00,  1.587715151679e+00, -9.944132184372e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.00010724231609 mm / 0.00671036134216 rad. After wrap: **0.00010724231609 mm / 0.00671036134216 rad**.

Max balanced Jacobian condition: 100.137; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_1_offset_-0.0001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.564756830421,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.534756830421e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9999229908e+02, -1.6795932577e-05,  2.9999987004e+02,  5.9440411499e-01,  1.5602929463e+00, -2.9445180586e-01]` | 1.504547297e-05 | 0.0002464234108 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -8.9283521374e-12,  3.0000000000e+02,  6.9999924941e-01,  1.5647567986e+00, -3.9999924975e-01]` | 8.960729912e-13 | 7.503044971e-11 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02, -1.1332740069e-13,  3.0000000000e+02,  6.9999999124e-01,  1.5647568301e+00, -3.9999999125e-01]` | 2.842170943e-14 | 8.750320943e-13 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  1.4376157189e-12,  3.0000000000e+02,  7.0472015484e-01,  1.5649568306e+00, -4.0471801488e-01]` | 2.223442518e-12 | 3.176187643e-13 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.318748127646,   5.976286300818, 299.713533664741,  -0.39698761444 ,
   1.541807864964,   0.669861857894]
before wrap : [ 2.999922990758e+02, -1.679593257717e-05,  2.999998700362e+02,
  5.944041149923e-01,  1.560292946330e+00, -2.944518058648e-01]
final joints: [ 2.999922990758e+02, -1.679593257717e-05,  2.999998700362e+02,
  5.944041149923e-01,  1.560292946330e+00, -2.944518058648e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 1.50454729727e-05 mm / 0.000246423410839 rad. After wrap: **1.50454729725e-05 mm / 0.000246423410839 rad**.

Max balanced Jacobian condition: 481.364; damping range 1e-10...0.0001; rejected steps: 45; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.318748127646,   5.976286300818, 299.713533664741,   2.74460503915 ,
   1.599784788626,  -2.471730795696]
before wrap : [ 2.995043012750e+02, -2.310098108493e-03,  2.999900394021e+02,
  1.240224647073e+00,  1.586525587305e+00, -9.399895950916e-01]
final joints: [ 2.995043012750e+02, -2.310098108493e-03,  2.999900394021e+02,
  1.240224647073e+00,  1.586525587305e+00, -9.399895950916e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.83952088893e-05 mm / 0.00597963160487 rad. After wrap: **7.83952088892e-05 mm / 0.00597963160487 rad**.

Max balanced Jacobian condition: 171.878; damping range 1e-08...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_1_offset_0.0 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.564856830421,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.534856830421e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9999235191e+02, -1.5885416793e-05,  2.9999987187e+02,  5.9205561292e-01,  1.5602934471e+00, -2.9210436513e-01]` | 1.48853886e-05 | 0.0002464878685 | iteration_limit; in limits: True |
| iterations_200 | `[ 2.9999996394e+02, -9.6143807575e-08,  2.9999999939e+02,  7.0634669645e-01,  1.5651257472e+00, -4.0634381902e-01]` | 1.220355209e-08 | 8.529076335e-07 | iteration_limit; in limits: True |
| iterations_200_tight | `[ 2.9999996394e+02, -9.6143807575e-08,  2.9999999939e+02,  7.0634669645e-01,  1.5651257472e+00, -4.0634381902e-01]` | 1.220355209e-08 | 8.529076335e-07 | iteration_limit; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  1.1001475334e-11,  3.0000000000e+02,  7.0002863787e-01,  1.5648580438e+00, -4.0002862488e-01]` | 8.267466377e-11 | 1.736558229e-11 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.318745987952,   5.976239890976, 299.713588499655,  -0.40006527719 ,
   1.541853197279,   0.672938256938]
before wrap : [ 2.999923519052e+02, -1.588541679286e-05,  2.999998718716e+02,
  5.920556129196e-01,  1.560293447104e+00, -2.921043651336e-01]
final joints: [ 2.999923519052e+02, -1.588541679286e-05,  2.999998718716e+02,
  5.920556129196e-01,  1.560293447104e+00, -2.921043651336e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 1.48853886008e-05 mm / 0.00024648786846 rad. After wrap: **1.4885388601e-05 mm / 0.00024648786846 rad**.

Max balanced Jacobian condition: 481.41; damping range 1e-10...0.0001; rejected steps: 45; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.318745987952,   5.976239890976, 299.713588499655,   2.741527376399,
   1.59973945631 ,  -2.468654396652]
before wrap : [ 2.995052018555e+02, -2.316181683520e-03,  2.999901040154e+02,
  1.237809512391e+00,  1.586523544925e+00, -9.375755688461e-01]
final joints: [ 2.995052018555e+02, -2.316181683520e-03,  2.999901040154e+02,
  1.237809512391e+00,  1.586523544925e+00, -9.375755688461e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.83605585317e-05 mm / 0.00597853844125 rad. After wrap: **7.83605585316e-05 mm / 0.00597853844125 rad**.

Max balanced Jacobian condition: 171.627; damping range 1e-08...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_1_offset_0.0001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.564956830421,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.534956830421e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9999191989e+02, -1.5636195509e-05,  2.9999986569e+02,  5.8534027162e-01,  1.5601103574e+00, -2.8539204349e-01]` | 1.784467674e-05 | 0.0002666423529 | iteration_limit; in limits: True |
| iterations_200 | `[ 2.9999996194e+02, -9.9360226842e-08,  2.9999999936e+02,  7.0458594884e-01,  1.5651511418e+00, -4.0458386975e-01]` | 1.644183673e-08 | 9.036582469e-07 | iteration_limit; in limits: True |
| iterations_200_tight | `[ 2.9999996194e+02, -9.9360226842e-08,  2.9999999936e+02,  7.0458594884e-01,  1.5651511418e+00, -4.0458386975e-01]` | 1.644183673e-08 | 9.036582469e-07 | iteration_limit; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  5.3124142470e-13,  3.0000000000e+02,  7.0000000015e-01,  1.5649568304e+00, -4.0000000015e-01]` | 8.020010148e-13 | 1.493134971e-14 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.31874385544 ,   5.976193481136, 299.713643334851,  -0.403152563885,
   1.541898254738,   0.676024279927]
before wrap : [ 2.999919198932e+02, -1.563619550905e-05,  2.999998656948e+02,
  5.853402716245e-01,  1.560110357359e+00, -2.853920434861e-01]
final joints: [ 2.999919198932e+02, -1.563619550905e-05,  2.999998656948e+02,
  5.853402716245e-01,  1.560110357359e+00, -2.853920434861e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 1.78446767421e-05 mm / 0.00026664235294 rad. After wrap: **1.78446767418e-05 mm / 0.00026664235294 rad**.

Max balanced Jacobian condition: 460.823; damping range 1e-10...0.0001; rejected steps: 45; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.31874385544 ,   5.976193481136, 299.713643334851,   2.738440089705,
   1.599694398852,  -2.465568373663]
before wrap : [ 2.994335204792e+02, -2.686143858712e-03,  2.999885034239e+02,
  1.268047962900e+00,  1.587685264294e+00, -9.678023928140e-01]
final joints: [ 2.994335204792e+02, -2.686143858712e-03,  2.999885034239e+02,
  1.268047962900e+00,  1.587685264294e+00, -9.678023928140e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.00010716031891 mm / 0.00670493429478 rad. After wrap: **0.00010716031891 mm / 0.00670493429478 rad**.

Max balanced Jacobian condition: 100.269; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_1_offset_0.001 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.565856830421,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.535856830421e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9999195977e+02, -5.6418695019e-06,  2.9999987592e+02,  5.5626258637e-01,  1.5597795344e+00, -2.5632747924e-01]` | 1.997131768e-05 | 0.0002935622693 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02, -5.5330503360e-14,  3.0000000000e+02,  7.0000000239e-01,  1.5658568305e+00, -4.0000000239e-01]` | 3.180819773e-13 | 2.393607308e-12 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02,  2.7312810839e-14,  3.0000000000e+02,  7.0000000005e-01,  1.5658568304e+00, -4.0000000005e-01]` | 2.295833841e-13 | 4.705380633e-14 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  5.5945325428e-14,  3.0000000000e+02,  7.0000000001e-01,  1.5658568304e+00, -4.0000000001e-01]` | 5.123796534e-14 | 1.277010422e-14 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.318724985996,   5.975775792758, 299.71413686419 ,  -0.431368660186,
   1.542291190873,   0.704229002911]
before wrap : [ 2.999919597714e+02, -5.641869501925e-06,  2.999998759170e+02,
  5.562625863655e-01,  1.559779534394e+00, -2.563274792364e-01]
final joints: [ 2.999919597714e+02, -5.641869501925e-06,  2.999998759170e+02,
  5.562625863655e-01,  1.559779534394e+00, -2.563274792364e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 1.99713176798e-05 mm / 0.000293562269261 rad. After wrap: **1.99713176798e-05 mm / 0.000293562269261 rad**.

Max balanced Jacobian condition: 428.298; damping range 1e-10...0.0001; rejected steps: 45; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.318724985996,   5.975775792758, 299.71413686419 ,   2.710223993403,
   1.599301462717,  -2.437363650679]
before wrap : [ 2.994430577886e+02, -2.750555565307e-03,  2.999891708877e+02,
  1.246389367921e+00,  1.587676855414e+00, -9.461536729445e-01]
final joints: [ 2.994430577886e+02, -2.750555565307e-03,  2.999891708877e+02,
  1.246389367921e+00,  1.587676855414e+00, -9.461536729445e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000106798156587 mm / 0.00668544946994 rad. After wrap: **0.000106798156587 mm / 0.00668544946994 rad**.

Max balanced Jacobian condition: 100.309; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## pole_1_offset_0.01 (near_singular)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.574856830421,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.544856830421e+00, -3.600000000000e-01]`

Selected branch: `1`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[ 2.9961534577e+02, -1.9095406371e-03,  2.9999614970e+02,  1.0306690348e+00,  1.5880369748e+00, -7.3052710520e-01]` | 7.932532811e-05 | 0.005539432389 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02,  1.1131346897e-13,  3.0000000000e+02,  7.0000000002e-01,  1.5748568304e+00, -4.0000000002e-01]` | 2.813605686e-13 | 1.71334551e-13 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02,  1.1131346897e-13,  3.0000000000e+02,  7.0000000002e-01,  1.5748568304e+00, -4.0000000002e-01]` | 2.813605686e-13 | 1.71334551e-13 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  9.4929802400e-14,  3.0000000000e+02,  7.0000000002e-01,  1.5748568304e+00, -4.0000000002e-01]` | 3.243687903e-13 | 1.687934801e-13 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.318568288295,   5.971598995681, 299.719073321582,  -0.75029029027 ,
   1.544795070294,   1.023036902477]
before wrap : [3.000408063604e+02, 1.415110245626e-03, 3.000006466867e+02,
 8.311519344111e-02, 1.548538586201e+00, 2.166056914712e-01]
final joints: [3.000408063604e+02, 1.415110245626e-03, 3.000006466867e+02,
 8.311519344111e-02, 1.548538586201e+00, 2.166056914712e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000345637883314 mm / 0.00202384167322 rad. After wrap: **0.000345637883315 mm / 0.00202384167322 rad**.

Max balanced Jacobian condition: 136.305; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.318568288295,   5.971598995681, 299.719073321582,   2.39130236332 ,
   1.596797583296,  -2.118555751113]
before wrap : [ 2.996153457729e+02, -1.909540637136e-03,  2.999961496960e+02,
  1.030669034778e+00,  1.588036974786e+00, -7.305271052021e-01]
final joints: [ 2.996153457729e+02, -1.909540637136e-03,  2.999961496960e+02,
  1.030669034778e+00,  1.588036974786e+00, -7.305271052021e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 7.93253281094e-05 mm / 0.0055394323888 rad. After wrap: **7.93253281095e-05 mm / 0.0055394323888 rad**.

Max balanced Jacobian condition: 98.9171; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.

## nominal_pole_1 (nominal_poles)

Target: `[300.            ,   0.            , 300.            ,   0.7           ,
   1.570796326795,  -0.4           ]`

Caller seed: `[ 3.001000000000e+02, -1.000000000000e-01,  3.001000000000e+02,
  7.500000000000e-01,  1.540796326795e+00, -3.600000000000e-01]`

Selected branch: `0`; strict pass: `False`.

Classification: **iteration_limit_without_final_validation**. The generating command is an in-limit exact solution.

| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |
| --- | --- | ---: | ---: | --- |
| default | `[3.0000802133e+02, 6.6435275973e-04, 3.0000026394e+02, 2.3754019184e-01, 1.5512915748e+00, 6.2252467117e-02]` | 0.0001226593722 | 0.00180004807 | iteration_limit; in limits: True |
| iterations_200 | `[ 3.0000000000e+02,  1.9187012020e-12,  3.0000000000e+02,  7.0000000083e-01,  1.5707963268e+00, -4.0000000083e-01]` | 3.165418046e-12 | 4.938887971e-12 | raw_residual_tolerance; in limits: True |
| iterations_200_tight | `[ 3.0000000000e+02,  2.2122667232e-14,  3.0000000000e+02,  7.0000000000e-01,  1.5707963268e+00, -4.0000000000e-01]` | 1.205831008e-13 | 2.823199168e-15 | raw_residual_tolerance; in limits: True |
| minimal_weighted_fast | `[ 3.0000000000e+02,  1.2624625547e-12,  3.0000000000e+02,  7.0000000068e-01,  1.5707963268e+00, -4.0000000068e-01]` | 1.888333327e-12 | 4.065872339e-12 | raw_residual_tolerance; in limits: True |

### Branch 0: nominal_0

```text
initial guess: [298.318631782161,   5.973483400402, 299.716845903252,  -0.599187772922,
   1.544008512232,   0.871985696121]
before wrap : [3.000080213304e+02, 6.643527597291e-04, 3.000002639377e+02,
 2.375401918351e-01, 1.551291574839e+00, 6.225246711687e-02]
final joints: [3.000080213304e+02, 6.643527597291e-04, 3.000002639377e+02,
 2.375401918351e-01, 1.551291574839e+00, 6.225246711687e-02]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 0.000122659372179 mm / 0.00180004807013 rad. After wrap: **0.000122659372179 mm / 0.00180004807013 rad**.

Max balanced Jacobian condition: 164.621; damping range 1e-08...0.0001; rejected steps: 46; forced accepts: 0.

### Branch 1: nominal_1

```text
initial guess: [298.318631782161,   5.973483400402, 299.716845903252,   2.542404880667,
   1.597584141358,  -2.269606957469]
before wrap : [ 2.995286713046e+02, -2.558042267838e-03,  2.999931936447e+02,
  1.124643883268e+00,  1.587682080377e+00, -8.244617874290e-01]
final joints: [ 2.995286713046e+02, -2.558042267838e-03,  2.999931936447e+02,
  1.124643883268e+00,  1.587682080377e+00, -8.244617874290e-01]
limit excess: [0., 0., 0., 0., 0., 0.]
```

Termination: iteration_limit; iterations: 50; final in limits: True.

Before wrap: 9.4403907169e-05 mm / 0.00620722410042 rad. After wrap: **9.44039071687e-05 mm / 0.00620722410042 rad**.

Max balanced Jacobian condition: 100.355; damping range 1e-07...0.0001; rejected steps: 47; forced accepts: 0.

Other solver settings and ablations: see this case in `traces.json.gz`.
