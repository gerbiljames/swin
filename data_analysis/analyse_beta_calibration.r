#!/usr/bin/env Rscript

library(stringr)
library(gtools)

colours <- c("blue", "green", "orange", "purple", "yellow",
 "black", "red", "pink", "brown", "lightblue")

logs_dir <- "../src/PiSwarmSimulator/logs/"

files <- list.files(path=logs_dir, pattern="^beta")

file_paths <- list.files(path=logs_dir, pattern="^beta", full.names=TRUE)

files <- mixedsort(files)

file_paths <- mixedsort(file_paths)

beta_data <- lapply(file_paths, read.csv, header=FALSE)

for (i in 1:length(beta_data)){
	colnames(beta_data[[i]]) <- c("time", "distance")
}

file_values <- sapply(files, str_sub, start=6, end=-5)

file_values <- sapply(file_values, strsplit, "_")

for (i in 1:length(file_values)){
	beta_data[[i]] <- c(beta_data[i], file_values[[i]])
}

sorted_beta_data <- list()

for (i in 1:length(beta_data)){

	sorted_index <- beta_data[[i]][[2]]

	current_index_contents <- sorted_beta_data[[sorted_index]]

	if (is.null(current_index_contents)) current_index_contents <- list()

	seed_value <- beta_data[[i]][[3]]

	data_frame <- beta_data[[i]][[1]]

	current_index_contents[[seed_value]] <- data_frame

	sorted_beta_data[[sorted_index]] <- current_index_contents

}

mean_beta_data <- list()

index_increment <- length(beta_data) / length(sorted_beta_data)

index <- 1

for (data_set in sorted_beta_data) {

	current_joined_set <- NULL

	for (data in data_set) {
		if (is.null(current_joined_set)) {
				current_joined_set <- data
			} else {
				current_joined_set <- merge(current_joined_set, data, by="time")
			}
	}

	mean_data_set <- data.frame(time=current_joined_set$time, mean_distance=rowMeans(current_joined_set[,-1]))

	mean_beta_data[[file_values[[index]][[1]]]] <- mean_data_set

	index <- index + index_increment

}

pdf("figures/beta_calibration.pdf")

plot(mean_beta_data[[1]], xlim=c(0, 15000), ylim=c(0, 500), type="n",
	xlab="Time (Seconds)", ylab="Centroid Distance from Beacon (cm)")

legend_text <- list()

index <- 1

for (i in 1:length(mean_beta_data)) {
	lines(mean_beta_data[[i]], col=colours[[i]])
	legend_text[[i]] <- file_values[[index]][[1]]

	index <- index + index_increment
}

legend("topright", col=colours, legend=legend_text, lty=1, title="beta")

rubbish <- dev.off()