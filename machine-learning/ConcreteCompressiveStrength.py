#!/usr/bin/env python
# coding: utf-8

# Michael DiGregorio
# <br>
# I pledge my honor that I have abided by the Stevens Honor System

# ## Hands-on

# To master your new-found knowledge of Python, try these hands-on examples. 
# 
# Your homeworks will be in a similar format to this section.
# 
# Consider the following CSV file "Concrete_Data.csv" containing the results of compressive tests for various types of concrete.

# In[ ]:


from sklearn import linear_model
import numpy as np
import seaborn as sns
import pandas as pd

# In[33]:


concrete_url = "./data/Concrete_Data.csv"


# **1. Load the CSV data into a pandas data frame. Print some high-level statistical info about the data frame's columns.**

# In[34]:


conc_data = pd.read_csv(concrete_url)
conc_data.tail(5)


# Below we can see some simple statistical data about the concrete dataset. Most notably, there are 1030 datapoints, standard devations are relatively large, and there are nine features

# In[35]:


conc_data.describe()


# Below we can see that of 1030 rows, only 379 of them show compressive strength values of above 40MPa

# **2. How many rows have a compressive strength > 40 MPa?**

# In[36]:


conc_high = conc_data[conc_data["Concrete_Compressive_Strength"] > 40]
conc_high["Concrete_Compressive_Strength"].count()


# **3. Plot the histogram of Coarse Aggregate and Fine Aggregate values**

# From the graphs below we can see a histogram of the frequency of Coarse and Fine Aggregate values. Both appear to have relatively large standard deviations, and the fine aggregate values seem to exhibit less kurtosis. 

# In[37]:


get_ipython().run_line_magic('matplotlib', 'inline')
conc_data.hist("Coarse_Aggregate")


# In[38]:


conc_data.hist("Fine_Aggregate")


# **4. Make a plot comparing compressive strength to age**

# Finally, when we compare compressive strength to age, we can see that as concrete ages it tends to get stronger and then begin to asymptote at a peak strength value. This "leveling off" appears to occur at approximately an age of 300.

# In[39]:


sns.jointplot(conc_data["Age"], conc_data["Concrete_Compressive_Strength"], kind="reg")


# **5. Make a plot comparing compressive strength to age for only those rows with < 750 fine aggregate.**

# This appears to indicate that even though concrete mixes with little fine aggregate experience an increase in compressive strength over time, the effect is not as large as it is with samples conatining a lot of fine aggregate. The "peak" also appears to come much sooner in the lifetime of the concrete

# In[40]:


conc_data_lfine = conc_data[conc_data["Fine_Aggregate"] < 750]
sns.jointplot(conc_data_lfine["Age"], conc_data_lfine["Concrete_Compressive_Strength"], kind="reg")


# **6. Try to build a linear model that predicts compressive strength given the other available fields.**
# 

# Below I construct a linear model to predict compressive strengeth based off of all of the other features available. I use 900 of the 1030 rows to train the model and 130 of the 1030 rows to test the model. This is done to prevent classifying an overfitted model as a more accurate model. 
# 
# The coefficients of the linear model output show that superplasticizer content and water content are the dominating factors in concrete compressive strength. The more superplasticizer and the less water there is within a given sample, the higher its compressive strength is likley to be.

# In[41]:


from sklearn import linear_model
concrete_model = linear_model.Lasso(alpha=.1)
clean_df = conc_data.dropna()
conc_features = conc_data.drop(columns="Concrete_Compressive_Strength")
target = conc_data["Concrete_Compressive_Strength"]


#break it up into training and test sets  10% is the test set and 90% is the train set
features_train = conc_features[:900]
target_train = target[:900]
features_test = conc_features[900:]
target_test = target[900:]

concrete_model.fit(features_train, target_train)
pd.DataFrame([dict(zip(features_train, concrete_model.coef_))])


# **7. Generate predictions for all the observations and a scatterplot comparing the predicted compressive strengths to the actual values.**

# Below is a comparison of predicted values to actual values for compressive strength. Only the first 5 are shown for brevity. The predicted values seem to be relatively close to the actual values, although for professional applications I would choose a more accurate model.

# In[42]:


predictions = concrete_model.predict(features_test)
test_df = clean_df[900:]
predictions_df = test_df.assign(predictions=predictions)
predictions_df[["Concrete_Compressive_Strength", "predictions"]].head(5)


# Below is a plot showing the relationship between the predicted and actual values for compressive strength. The relationship is surprisingly linear (perfect linearity implies perfect predictions) considering the kind of data being worked with and the simplicity of the model. This shows that some of the features may have a relationship with compressive strength that could be more easily expressed with a nonlinear model

# In[43]:


sns.jointplot("Concrete_Compressive_Strength", "predictions", predictions_df, kind="reg")


# In[ ]:




