 namespace KalmanFilter
 module MarketsInteractions =
    open Data
    open MathNet.Numerics.LinearAlgebra
    open System
    open KalmanFilter
    open MathNet.Numerics.LinearAlgebra.DenseMatrix
    open Helpers
    
    let logLikelihood kalmanInput (kalmanTraces:KalmanTrace seq) =
        /// Calculates the change of the log likelihood after an update.
        /// Estimates the likelihood of the actual 'observedValue' with respect 
        /// to our 'projectedValue' given 'projectedVar' variance
        let logLikelihoodChange indexCount' kalmanTrace =
            let indexCount = indexCount' + 1
            // Projected probability with added noise
            let S : Matrix<float> = kalmanTrace.ProjectedVar + kalmanInput.CovarMeasurementsNoise
            // Calculate log likelihood change
            let d = kalmanTrace.ObservedValue.Column(0) - kalmanTrace.ProjectedState.Column(0)
            let SDeterminant = S.Determinant()
            let SInverse = S.Inverse()
            -(float indexCount)*0.5*log(2.0*Math.PI)
                - 0.5*log(SDeterminant) - 0.5*(d * SInverse)*d
        Seq.mapi logLikelihoodChange kalmanTraces
        |> Seq.sum

    let expectationStep kalmanInput =
        let kalmanTraces = kalmanFilter2 kalmanInput
        let loglikelihood = logLikelihood kalmanInput kalmanTraces
        kalmanTraces, loglikelihood
 
    /// Represents the 'maximization' step of the algorithm
    /// (Compute new value of parameters to maximize likelihood)
    let maximizationStep (kalmanTraces:KalmanTrace seq) (kalmanInput:KalmanInput) =
        let dynamics = kalmanInput.StatesTransition
        let nextValues =
            KalmanTrace.NextValues kalmanTraces
            |> Seq.map (fun valueAsMatrix -> valueAsMatrix.Column(0))
        let variances =
            KalmanTrace.NextVariances kalmanTraces
        let observedCount = Seq.length kalmanTraces

        // Transform hidden values into a matrix & get components
        let hiddenVals = ofColumns (Seq.toList nextValues)
        let hiddenPrev = hiddenVals.[0 .., 0 .. observedCount - 2] 
        let hiddenNext = hiddenVals.[0 .., 1 .. ] 

        // Calculate variance between two neighboring elements
        let crossVar = 
            Seq.pairwise variances
            |> Seq.map (fun (pastVar, nextVar) ->
                (dynamics * pastVar).Transpose() * ((dynamics * pastVar) * dynamics.Transpose()).Inverse() * nextVar)
            |> Seq.reduce (+) 

        // Sum variance matrices excluding the last one
        let vars = 
            variances 
            |> Seq.take (observedCount - 1) |> Seq.reduce (+)

        // Calculate new value of 'dynamics' parameter
        let h1 = (hiddenPrev * hiddenNext.Transpose()) + crossVar
        let h2 = (hiddenPrev * hiddenPrev.Transpose()) + vars
        h1.Transpose() * h2.Inverse() // new states transition

    let kalmanInputFor (trainingData:Matrix<float>) transitionMatrix =
        let indexCount = trainingData.RowCount
        let idMatrix = idMatrix indexCount
        let initialState = [trainingData.Column(0)] |> ofColumns
        {
            TrainingData = trainingData
            InitialState = initialState
            StatesTransition = transitionMatrix
            MeasurementModel = idMatrix
            CovarProcessNoise = 0.01 * idMatrix
            CovarMeasurementsNoise = 0.01 * idMatrix
        }

    /// Repeatedly runs expectation and maximization step of
    /// the EM algorithm to calculate the 'dynamics' parameter
    let fitModel (trainingData:Matrix<float>) maxIter =

        let indexCount = trainingData.RowCount
        let idMatrix = idMatrix indexCount
        let kalmanInput = kalmanInputFor trainingData idMatrix
        /// Inner recursive function that is called recursively
        let rec updateModel dynamics (logliks:float list) iter =
            let kalmanInput = { kalmanInput with StatesTransition = dynamics }
            // Run a single step of the EM algorithm
            let kalmanTraces, loglikelihood = expectationStep kalmanInput
            let newDynamics = maximizationStep kalmanTraces kalmanInput

            // Check for convergence
            let logliks = loglikelihood::logliks
            match logliks with
            | current::past::_ when current - past < 0.1 -> 
                newDynamics, List.rev logliks, Some iter
            | _ when iter > maxIter ->
                newDynamics, List.rev logliks, None
            | _ ->
                updateModel newDynamics logliks (iter + 1)

        // Start the estimation with the identity matrix
        updateModel idMatrix [] 1

    let main (names:string[]) trainingData validationData =
        let dynamics, logliks, converged = fitModel trainingData 100

        // Report whether the training has converged
        match converged with
        | Some iter -> printfn "Converged in iteration %i" iter
        | _ -> printfn "Not converged."

        // Print interactions between pairs of stock markets
        dynamics |> Matrix.iteri (fun i j v -> 
          if i <> j && (abs v > 0.2) then 
            printfn "Interaction between %s and %s: %f" 
                    names.[i] names.[j] v)

        expectationStep (kalmanInputFor trainingData dynamics)