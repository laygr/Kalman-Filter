namespace KalmanFilter
module Program =
    open System
    open FSharp.Data
    open FSharp.Data.CsvExtensions
    open Data
    open FSharp.Charting

    [<EntryPoint>]
    let main argv =
        let cpiUrl = "C:\Users\laygr\Desktop\CPI Suecia.csv"
        let csv = CsvFile.Load(cpiUrl,hasHeaders=true)
        let names = [| "GRCP20YY"; "BECPYOY"; "NECPIYOY"; "DNCPIYOY"; "NOCPIYOY"; "Co1"; "EURSEK" |]
        let trainingData, validationData =
            prepareData2 csv names
            |> lastColumns 100
            |> normalize -1.0 1.0
            |> splitData
        let traces, logliks = MarketsInteractions.main names trainingData validationData
        
        let simpleLine name values =
            Chart.Line(data=Seq.mapi (fun i v -> (string i), v) values,  Name=name)
        
        logliks
        |> simpleLine "Log Likelihood"

        
        0 // return an integer exit code