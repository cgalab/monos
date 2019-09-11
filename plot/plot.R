library(ggplot2)
library(scales)
library(reshape2)

filenames <- list.files(path = "~/devel/ord/monos/plot/", pattern = "*.csv", full.names = TRUE)

df <- data.frame()

for(file in filenames) {
  dat <- read.csv(file, header = F, na.strings = "")
  colnames(dat)[1] <- "SIZE"
  colnames(dat)[2] <- "TIME"
  colnames(dat)[3] <- "Filename"
  if(nrow(df) == 0) {
    df <- dat
  } else {
    df <- rbind(df,dat)  
  }
}

df <- df[complete.cases(df),]
df <- df[order(df$SIZE),]

#df <- merge(fist.regular,fist.regularcdt,by="FILENAME")
#df = merge(df,fist.triangle,by="FILENAME")

# points 
pdf("monos-plot.pdf", width = 3.7, height = 3.7)
ggplot(data = df, aes(x = SIZE)) +
  #  geom_point(aes(y = SpeedupDC16, colour = "FIST D&C 16   "), fill=NA, size=.7, alpha=.7) +
  geom_point(aes(y = TIME, colour = "Monos   "), fill=NA, size=.7, alpha=.7) +
  #geom_point(aes(y = SpeedupDC4, colour = "FIST D&C 4   "), fill=NA, size=.7, alpha=.7) +
  #geom_point(aes(y = SpeedupDC2, colour = "FIST D&C 2   "), fill=NA, size=.7, alpha=.7) +
  geom_line(y=1) +  
  scale_colour_manual("", 
                      breaks = c("Monos ", "FIST D&C 2   ", "FIST D&C 4   ", "FIST D&C 8   ", "FIST D&C 16   "
                                 , "FIST(P&C 32)   ", "FIST(P&C 64)   "),
                      values = c("green", "blue", "red", "magenta", "orange", "black")) +
  xlab("# vertices") +
  ylab("time [sec]") + 
  coord_cartesian(xlim=c(1,1000), ylim=c(0, 50)) +
  guides(colour = guide_legend(override.aes = list(size = 5, alpha=1)))+
  theme_bw() + 
  theme(#plot.background = element_blank(),panel.grid.major = element_blank(),
    #panel.grid.minor = element_blank(),panel.border = element_blank(),    
    legend.key = element_blank(),
    legend.position="bottom"
    #legend.key=element_rect(fill=NA)
    #     legend.position="none"
  ) +
#  scale_x_log10(labels = comma)
   scale_x_continuous(labels = comma)  

dev.off()


