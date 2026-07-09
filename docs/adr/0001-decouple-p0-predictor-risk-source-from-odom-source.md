# Decouple P0 predictor risk source from odom source

P0 RiskGridMap must be able to keep GNSS-assisted odometry while evaluating grid risk from a LiDAR-dominant predictor source. We decided to expose explicit P0 Predictor source mode and GNSS epoch policy parameters, with strict invalid-value failures, so experiments can prove which advisory sources were used instead of inferring them from odom or integrity-fusion settings.
