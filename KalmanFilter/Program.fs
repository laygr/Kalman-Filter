namespace KalmanFilter

module Program =
    open System
    open DataHandling
    open FSharp.Data
    open FSharp.Data.CsvExtensions
    open MathNet.Numerics.LinearAlgebra

    let test (names:string[]) data dynamics =
        // Run the E step using the calculated 'dynamics'
        let testOutput = MarketsInteractions.onlineLearning data dynamics
        let timeSteps = [0..data.ColumnCount-1]
        (testOutput.SelectByIndex 7).ToDeedleDataFrame(timeSteps)
        |> AnalyzeResults.plotFrame
        //let traces, logLiks = MarketsInteractions.expectationStep (MarketsInteractions.kalmanInputFor data dynamics)


    [<EntryPoint>]
    let main argv =
        let cpiUrl = "/Users/laygr/Desktop/CPI Suecia Para Kalman.csv"
        let csv = CsvFile.Load(cpiUrl,hasHeaders=true)
        let names = [| "GRCP20YY"; "BECPYOY"; "NECPIYOY"; "DNCPIYOY"; "NOCPIYOY"; "Co1"; "EURSEK"; "SWCPYOY" |]
        let fullDataUnormalized = 
            prepareData2 csv names
            |> lastColumns 100
        let fullDataNormalized =
            fullDataUnormalized
            |> normalize -1.0 1.0
        let trainingData, validationData =
            fullDataUnormalized
            |> splitData
        let dynamics, traces, logliks = MarketsInteractions.main names trainingData
        (*
        let simpleLine name (values:float seq) =
            Chart.Line(data=Seq.mapi (fun i v -> v) values)
            |> Chart.WithTitle name

        logliks
        |> simpleLine "Log Likelihood"
        |> Chart.Show
        *)
        test names fullDataUnormalized dynamics

        0 // return an integer exit code
