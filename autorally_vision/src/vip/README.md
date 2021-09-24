# autorally-traversability
Development on GT's small scale autonomous platform AutoRally, specifically on terrain traversability analysis

# How to run

1. Ensure you have pipenv installed:

```
pip install pipenv
```

2. in this directory (`/vip`) sync dependencies

```
pipenv sync
```

3. Run the labeling script
```
pipenv run python3 labeling.py
```

## bag files tested
`alpha_autorally0_2020-07-23-16-27-57_0.bag`
- missing camera intrinsics matrix
- has weird state estimator issues with z
 
`alpha_autorally3_2021-08-19-11-21-12.bag`
- still testing

`beta_autorally4_2021-02-21-18-30-44_0.bag`
- still testing

`platformA_2015-08-13-12-02-41_split1.bag`
- still testing

