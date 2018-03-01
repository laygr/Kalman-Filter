 namespace KalmanFilter
 module AnalyzeResults =
    open MathNet.Numerics.LinearAlgebra
    open System
    open KalmanFilter
    open MathNet.Numerics.LinearAlgebra.DenseMatrix
    open Helpers
    open XPlot.GoogleCharts
    open Deedle
    open Deedle.``F# Frame extensions``
    open XPlot.GoogleCharts.Deedle

    let exportToCsv (frame:Frame<_,_>) filePath =
        frame.SaveCsv(path=filePath)

    let plotFrame (frame:Frame<_,_>) =
        Chart.Line(frame)
        |> Chart.Show

    module SingleSeries =

        type TestOutput = {
            YTest: float[]
            YPrediction: float[]
        } with
            member x.PlotRealVsPredicted title xAxisLabels =
                [x.YTest; x.YPrediction]
                |> Seq.map (Seq.zip xAxisLabels)
                |> Chart.Combo 
                |> Chart.WithLabels ["Real"; "Predicted"]
                |> Chart.WithTitle title
                |> Chart.Show
            member x.ToDeedleDataFrame(labels:int list) =
                let yTest = Deedle.Series(labels, Array.toList x.YTest)
                let yPrediction = Deedle.Series(labels, x.YPrediction)

                Frame<_,_>(["Real"; "Predicted"], [yTest; yPrediction])

    module MultipleSeries =

        type TestOutput = {
            YTestV : Vector<float> []
            YPredictionV : Vector<float>[]
        } with
            member x.SelectByIndex (index:int) =
                {
                    YTest = Array.map (fun (v:Vector<float>) -> v.[index]) x.YTestV
                    YPrediction = Array.map (fun (v:Vector<float>) -> v.[index]) x.YPredictionV
                } : SingleSeries.TestOutput

            member x.plotRealVsPredicted index xAxisLabels =
                (x.SelectByIndex index).PlotRealVsPredicted xAxisLabels