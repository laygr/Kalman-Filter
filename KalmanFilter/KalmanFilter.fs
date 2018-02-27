namespace KalmanFilter
module KalmanFilter =
    open System
    open MathNet.Numerics
    open MathNet.Numerics.LinearAlgebra
    open MathNet.Numerics.LinearAlgebra.Double
    open MathNet.Numerics.LinearAlgebra.MatrixExtensions
    open Helpers
    open MathNet.Numerics.LinearAlgebra
    open MathNet.Numerics.LinearAlgebra.DenseMatrix

    open MathNet.Filtering.Kalman
    open KalmanFilter

    type KalmanInput = {
        TrainingData : Matrix<float>            // vectors (observable variables)
        InitialState : Matrix<float>            // vector  (the initial hidden state)
        StatesTransition : Matrix<float>        // matrix (states transition model)
        MeasurementModel : Matrix<float>        // matrix (observation model)
        CovarProcessNoise : Matrix<float>       // matrix (noise Q)
        CovarMeasurementsNoise : Matrix<float>  // matrix (noise R)
    } with
        member x.ObservedCount = x.TrainingData.RowCount
        member x.IndexCount = x.TrainingData.ColumnCount

    /// Represents the results of single iteration
    type KalmanTrace = {
        ProjectedState : Matrix<float> // vector
        ProjectedVar : Matrix<float>   // matrix?
        ObservedValue : Matrix<float>  // vector
        UpdatedState : Matrix<float>
        NextVar : Matrix<float>
        //NextValue : Matrix<float>
    } with
        static member NextValues traces = Seq.map (fun kalmanTrace -> kalmanTrace.UpdatedState) traces
        static member NextVariances traces = Seq.map (fun kalmanTrace -> kalmanTrace.NextVar) traces

    /// Calculates values projected by the Kalman filter model
    // covarProcessNoise: Q
    // covarMeasurementsNoise: R
    let kalmanFilter1 kI =
        let k = DiscreteKalmanFilter(kI.InitialState, kI.CovarProcessNoise)
        k.Predict(kI.StatesTransition, kI.CovarProcessNoise)
        let step observedValue pastValue =
            // Calculate projected value and its variance
            // A priori estimation,
            // Covar of the error of the a priori estimation
            let projectedState, projectedVar =
                k.Predict(kI.StatesTransition, kI.CovarProcessNoise) // don't use covarMeasurementsNoise as in tryF#
                k.State, k.Cov
            // Calculate projected value and its variance
            //let projectedValue = projectedState * pastValue
            // 2) Measurement Update ("Correct")
            let updatedProjectedState, nextVar =
                k.Update(observedValue, kI.MeasurementModel, kI.CovarMeasurementsNoise)
                k.State, k.Cov
            //let nextValue = updatedProjectedState * pastValue

            {
                ProjectedState = projectedState
                ProjectedVar = projectedVar
                ObservedValue = observedValue
                UpdatedState = updatedProjectedState
                //NextValue = nextValue
                NextVar = nextVar
            }

        // Size of the observedData matrix for future use
        seq {
            for i in [1 .. kI.ObservedCount - 1] do
                let observedValue = ofColumns ([kI.TrainingData.Column(i)])
                let previousValue = ofColumns ([kI.TrainingData.Column(i-1)])
                yield (step observedValue previousValue)
        }

    let kalmanFilter2  (kI:KalmanInput) =
        let rec kalmanFilter2' timeStep traces =
            if timeStep = kI.ObservedCount
            then List.rev traces
            else
                let dynamics = kI.StatesTransition
                let noiseQ = kI.CovarProcessNoise
                let noiseR = kI.CovarMeasurementsNoise
                let observedValue = kI.TrainingData.Column(timeStep)
                let pastValue, pastVar = 
                  if timeStep = 0 then kI.TrainingData.Column(0), noiseQ
                  else
                    let lastTrace = List.head traces
                    lastTrace.ProjectedState.Column(0), lastTrace.NextVar

                // Calculate projected value and its variance
                let projectedValue = dynamics * pastValue
                let projectedVar = dynamics * pastVar * dynamics.Transpose() + noiseQ

                // Calculate 'Kalman gain' and update the values using observed data
                let kalmanGain = projectedVar * (projectedVar + noiseR).Inverse()
                let update = kalmanGain * (observedValue - projectedValue)
                let nextValue = projectedValue + update
                let nextVar = projectedVar - kalmanGain * projectedVar |> symmetrize

                // Compute the state for the next step of the iteration
                //let logChange = logLikelihoodChange observedValue projectedValue projectedVar
                let trace = {
                    ProjectedState = ofColumns [projectedValue]
                    ProjectedVar = projectedVar
                    ObservedValue = ofColumns [observedValue]
                    UpdatedState = ofColumns [nextValue]
                    //NextValue = nextValue
                    NextVar = nextVar
                }
                kalmanFilter2' (timeStep + 1) (trace::traces)
        kalmanFilter2' 0 []